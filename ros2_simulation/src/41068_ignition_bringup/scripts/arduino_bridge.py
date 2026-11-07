#!/usr/bin/env python3
"""TRACE operator box: E-STOP button, IR motion sensor and green LED on an Arduino Nano.

The Arduino (arduino/trace_estop/trace_estop.ino) sends one line per event over USB serial:
    STOP BUTTON   button pressed              -> stop every robot
    STOP IR       IR sensor saw movement      -> stop the drones (and the Husky)
    START         button held 1 s while stopped -> start every robot again
    BLOCKED IR    start refused: the sensor still sees movement
This node turns them into the same messages the dashboard's Stop/Start buttons
publish (True = stop), so mission control forwards them to the robots and the
dashboard's buttons update by themselves.

It also tells the Arduino when the mission is complete, so the green LED blinks:
mission DONE (mission control has checked the Husky is within 3 m of the person),
and every drone hovering (HOLDING) within arrive_radius_m of the person.

    sudo apt install python3-serial          # once
    ros2 run 41068_ignition_bringup arduino_bridge.py --ros-args -p port:=/dev/ttyUSB0

Parameters:
    port            serial port (Nano: usually /dev/ttyUSB0, Uno: /dev/ttyACM0)
    ir_enabled      false = ignore the IR sensor (e.g. a busy demo room)
    ir_stops_husky  false = IR movement stops only the drones
    arrive_radius_m how close a drone must hover to the person to count as arrived
"""
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError:
    serial = None

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
DRONES = ('parrot1', 'parrot2')   # parrot2 only exists with --second-drone
HUSKY = 'husky1'


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baud = int(self.declare_parameter('baud', 115200).value)
        self.ir_enabled = bool(self.declare_parameter('ir_enabled', True).value)
        self.ir_stops_husky = bool(self.declare_parameter('ir_stops_husky', True).value)
        self.arrive_radius = float(self.declare_parameter('arrive_radius_m', 7.0).value)

        self.link = None
        self.buffer = b''
        self.state = 'unknown'          # /mission/state
        self.husky_status = 'unknown'   # /mission/husky_status
        self.casualty = None            # (x, y), world frame
        self.drone_status = {}          # only drones that exist report a status
        self.drone_pos = {}
        self.complete = False

        # Same topics as the dashboard's Stop/Start buttons (True = stop).
        self.stop_pubs = {name: self.create_publisher(Bool, f'/mission_control/{name}/estop', 10)
                          for name in (HUSKY,) + DRONES}

        self.create_subscription(String, '/mission/state', self.on_state, LATCHED)
        self.create_subscription(String, '/mission/husky_status', self.on_husky_status, LATCHED)
        self.create_subscription(PointStamped, '/mission/casualty', self.on_casualty, LATCHED)
        for name in DRONES:
            self.create_subscription(String, f'/{name}/mission/status',
                                     lambda msg, n=name: self.on_drone_status(n, msg), LATCHED)
            self.create_subscription(Odometry, f'/{name}/odometry',
                                     lambda msg, n=name: self.on_drone_odom(n, msg),
                                     qos_profile_sensor_data)

        self.create_timer(0.02, self.read_serial)
        self.create_timer(1.0, self.keep_alive)
        if serial is None:
            self.get_logger().error('pyserial missing: sudo apt install python3-serial')
        self.get_logger().info(
            f'Arduino bridge on {self.port}: IR sensor {"on" if self.ir_enabled else "off"}, '
            f'IR stops {"drones + Husky" if self.ir_stops_husky else "drones only"}')

    # ---------------- serial link ----------------

    def connect(self):
        if serial is None:
            return
        try:
            self.link = serial.Serial(self.port, self.baud, timeout=0)
            time.sleep(2.0)   # opening the port resets the Nano; wait for it to boot
            self.link.reset_input_buffer()
            self.get_logger().info(f'Connected to the Arduino on {self.port}')
            self.send_settings()
        except (serial.SerialException, OSError) as err:
            self.link = None
            self.get_logger().warn(f'No Arduino on {self.port} ({err}); retrying',
                                   throttle_duration_sec=10.0)

    def send(self, data):
        if self.link is None:
            return
        try:
            self.link.write(data)
        except (serial.SerialException, OSError):
            self.get_logger().warn('Arduino disconnected')
            self.link = None

    def send_settings(self):
        # Resent every second: harmless if unchanged, and covers an Arduino reset.
        self.send(b'I' if self.ir_enabled else b'i')
        self.send(b'D' if self.complete else b'O')

    def keep_alive(self):
        if self.link is None:
            self.connect()
        else:
            self.send_settings()

    def read_serial(self):
        if self.link is None:
            return
        try:
            self.buffer += self.link.read(256)
        except (serial.SerialException, OSError):
            self.get_logger().warn('Arduino disconnected')
            self.link = None
            return
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            self.on_line(line.decode(errors='ignore').strip())

    # ---------------- Arduino -> robots ----------------

    def on_line(self, line):
        if line == 'STOP BUTTON':
            self.publish_stop(True, (HUSKY,) + DRONES)
            self.get_logger().warn('E-STOP button: stopping every robot')
        elif line == 'STOP IR':
            if not self.ir_enabled:
                return
            robots = ((HUSKY,) if self.ir_stops_husky else ()) + DRONES
            self.publish_stop(True, robots)
            self.get_logger().warn('IR sensor: movement detected, stopping '
                                   + ('every robot' if self.ir_stops_husky else 'the drones'))
        elif line == 'START':
            self.publish_stop(False, (HUSKY,) + DRONES)
            self.get_logger().info('Start button: every robot moving again')
        elif line == 'BLOCKED IR':
            self.get_logger().warn('Start refused: the IR sensor still sees movement')
        elif line in ('READY', 'IR ARMED'):
            self.get_logger().info(f'Arduino: {line}')

    def publish_stop(self, stop, robots):
        for name in robots:
            self.stop_pubs[name].publish(Bool(data=stop))

    # ---------------- mission -> green LED ----------------

    def on_state(self, msg):
        self.state = msg.data
        self.update_complete()

    def on_husky_status(self, msg):
        self.husky_status = msg.data
        self.update_complete()

    def on_casualty(self, msg):
        if self.casualty is None or self.state == 'SEARCHING':
            self.casualty = (msg.point.x, msg.point.y)

    def on_drone_status(self, name, msg):
        self.drone_status[name] = msg.data
        self.update_complete()

    def on_drone_odom(self, name, msg):
        p = msg.pose.pose.position
        self.drone_pos[name] = (p.x, p.y)
        if self.state == 'DONE':
            self.update_complete()

    def drone_arrived(self, name):
        pos = self.drone_pos.get(name)
        if self.drone_status.get(name) != 'HOLDING' or pos is None or self.casualty is None:
            return False
        return math.hypot(pos[0] - self.casualty[0], pos[1] - self.casualty[1]) <= self.arrive_radius

    def update_complete(self):
        if self.state != 'DONE':
            complete = False
        elif self.complete:
            # Latched: at DONE the drones get re-tasked to their overwatch
            # points, and the light must not flicker off while they reposition.
            complete = True
        else:
            complete = (self.husky_status == 'ARRIVED' and len(self.drone_status) > 0
                        and all(self.drone_arrived(name) for name in self.drone_status))
        if complete != self.complete:
            self.complete = complete
            self.send(b'D' if complete else b'O')
            if complete:
                self.get_logger().info('Mission complete: Husky and drones at the casualty, green LED blinking')


def main():
    rclpy.init()
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.link is not None:
            node.link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
