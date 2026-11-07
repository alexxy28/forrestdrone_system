#ifndef DATA_STRUCTURES_HPP
#define DATA_STRUCTURES_HPP

// Everything two or more nodes must agree on, plus every topic name.


struct Waypoint{
    double x;
    double y;
};

enum class HuskyStatus { IDLE, EN_ROUTE, STOPPED, ARRIVED, FAILED };

/// The text published on topics::HUSKY_STATUS.
inline const char* toString(HuskyStatus status){
    switch (status) {
        case HuskyStatus::IDLE:     return "IDLE";
        case HuskyStatus::EN_ROUTE: return "EN_ROUTE";
        case HuskyStatus::STOPPED:  return "STOPPED";
        case HuskyStatus::ARRIVED:  return "ARRIVED";
        case HuskyStatus::FAILED:   return "FAILED";
    }
    return "UNKNOWN";
}

namespace topics {
// Mission: created by these nodes.
inline constexpr char ROUTE[]        = "/mission/route";         // nav_msgs/Path, control -> husky
inline constexpr char HUSKY_STATUS[] = "/mission/husky_status";  // std_msgs/String, husky -> control
inline constexpr char CASUALTY[]     = "/mission/casualty";      // geometry_msgs/PointStamped, parrot -> control
inline constexpr char CASUALTY_CANDIDATE[] = "/mission/casualty_candidate";  // geometry_msgs/PointStamped, parrot -> UI, latched (detection awaiting review)
inline constexpr char STATE[]        = "/mission/state";         // std_msgs/String, control -> all
inline constexpr char HUSKY_HOLD[]   = "/mission/husky_hold";    // geometry_msgs/PointStamped, husky -> UI, latched (verified stop position, world)

// Operator (rqt UI) <-> robots.
inline constexpr char CASUALTY_DISCOVERED[] = "/CasulityDiscovered";  // std_msgs/Bool, parrot -> UI, latched
inline constexpr char CASUALTY_CONFIRMED[]  = "/CasulityConfirmed";   // std_msgs/Bool, UI -> parrot, latched
inline constexpr char UI_HUSKY_ESTOP[]  = "/mission_control/husky1/estop";   // std_msgs/Bool, UI -> control
inline constexpr char UI_PARROT_ESTOP[] = "/mission_control/parrot1/estop";  // std_msgs/Bool, UI -> control
inline constexpr char HUSKY_ESTOP[]     = "/husky1/estop";    // std_msgs/Bool, control -> husky (and UI label)
inline constexpr char PARROT_ESTOP[]    = "/parrot1/estop";   // std_msgs/Bool, control -> parrot
// Second drone (second_drone:=true). The dashboard's Stop/Start Parrot button
// drives both drones; this UI topic stops/starts parrot2 on its own.
inline constexpr char UI_PARROT2_ESTOP[] = "/mission_control/parrot2/estop";  // std_msgs/Bool, UI/terminal -> control
inline constexpr char PARROT2_ESTOP[]    = "/parrot2/estop";   // std_msgs/Bool, control -> parrot2

// Parrot: provided by the simulator and its helper nodes.
// Raw odometry is in world coordinates; the EKF output (/parrot1/odom) sits metres off in x/y.
inline constexpr char PARROT_ODOM[]    = "/parrot1/odometry";        // nav_msgs/Odometry, raw Gazebo, sim -> parrot
inline constexpr char PARROT_CMD_VEL[] = "/parrot1/cmd_vel";         // geometry_msgs/Twist, parrot -> sim
inline constexpr char PARROT_SCAN[]    = "/parrot1/scan/points";     // sensor_msgs/PointCloud2, sim -> terrain_map
inline constexpr char PARROT_TERRAIN[] = "/parrot1/terrain_points";  // sensor_msgs/PointCloud2 (x, y, z, sigma, count), terrain_map -> control
inline constexpr char PARROT_TERRAIN_SEED[] = "/parrot1/terrain_seed";  // same format, saved map (ros2 bag) -> terrain_map, first message only
inline constexpr char PARROT_TF[]        = "/parrot1/tf";            // tf2_msgs/TFMessage, sim -> terrain_map
inline constexpr char PARROT_TF_STATIC[] = "/parrot1/tf_static";     // tf2_msgs/TFMessage, sim -> terrain_map
inline constexpr char PARROT_THERMAL[] = "/parrot1/thermal_camera";  // sensor_msgs/Image, sim -> parrot
inline constexpr char PARROT_DEPTH[]   = "/parrot1/camera/depth/image";  // sensor_msgs/Image, sim -> parrot

// Second drone: same airframe, raw odometry also in world coordinates (frame parrot2_odom).
inline constexpr char PARROT2_ODOM[] = "/parrot2/odometry";     // nav_msgs/Odometry, sim -> terrain_map
inline constexpr char PARROT2_SCAN[] = "/parrot2/scan/points";  // sensor_msgs/PointCloud2, sim -> terrain_map
inline constexpr char PARROT2_MISSION_PATH[]   = "/parrot2/mission/path";    // nav_msgs/Path, control -> parrot2
inline constexpr char PARROT2_MISSION_STATUS[] = "/parrot2/mission/status";  // std_msgs/String, parrot2 -> control

// Husky: provided by the simulator and Nav2.
inline constexpr char HUSKY_ODOM[]      = "/husky1/odometry";          // nav_msgs/Odometry, raw Gazebo (world frame), sim -> control + husky
inline constexpr char HUSKY_NAVIGATE[]  = "/husky1/navigate_to_pose";  // nav2_msgs/NavigateToPose action, husky -> Nav2
inline constexpr char HUSKY_TF[]        = "/husky1/tf";                // tf2_msgs/TFMessage, sim + Nav2 -> husky
inline constexpr char HUSKY_TF_STATIC[] = "/husky1/tf_static";         // tf2_msgs/TFMessage, sim -> husky
}

#endif
