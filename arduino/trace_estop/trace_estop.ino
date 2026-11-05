// TRACE operator box: hardware E-STOP button, IR motion sensor, green status LED.
// Board: Arduino Nano (ATmega328P). Talks to arduino_bridge.py over USB serial.
//
// Wiring (A0-A5 work as normal digital pins on the Nano):
//   IR sensor:  OUT -> A2, VCC -> 5V, GND -> GND
//   Green LED:  A3 -> 220 ohm resistor -> LED long leg; LED short leg -> GND
//   Button:     A4 -> button -> GND (internal pull-up, no resistor needed)
//   Built-in LED (pin 13, on the board): on while the system is stopped
//
// Button:  a press STOPS every robot, every time (even if this board thinks
//          they are already stopped, so it can never miss a real stop).
//          To START again: hold it for 1 s while stopped.
// IR:      movement stops the robots, and the green LED flashes fast.
//          Start is refused while the sensor still sees movement.
// Green:   fast flash = stopped by the IR sensor
//          slow blink = mission complete (Husky and drones at the person)
//          off        = anything else
//
// USB serial, 115200 baud:
//   sends     READY, IR ARMED, STOP BUTTON, STOP IR, START, BLOCKED IR
//   receives  'D' mission complete, 'O' not complete,
//             'I' IR sensor on, 'i' IR sensor off

const int IR_PIN = A2;
const int LED_PIN = A3;
const int BUTTON_PIN = A4;
const int STOP_LED_PIN = LED_BUILTIN;

// PIR sensor (HC-SR501): OUT goes HIGH on movement -> true.
// IR obstacle module (FC-51, "OUT" goes LOW when it sees something) -> false.
const bool IR_ACTIVE_HIGH = true;
// Button wired A4 -> GND -> true. Wired 5V -> A4 with a resistor A4 -> GND -> false.
const bool BUTTON_ACTIVE_LOW = true;

const unsigned long DEBOUNCE_MS = 50;
const unsigned long HOLD_TO_START_MS = 1000;  // set 0 for a plain press
const unsigned long IR_WARMUP_MS = 30000;     // a PIR gives false triggers for ~30 s after power-up
const unsigned long IR_FLASH_MS = 100;        // fast flash: 5 per second
const unsigned long DONE_BLINK_MS = 500;      // slow blink: 1 per second

bool stopped = false;
bool irAlarm = false;       // stopped by the IR sensor, until START
bool irEnabled = true;      // the bridge can switch the sensor off ('i')
bool irArmed = false;       // warm-up over
bool missionDone = false;   // from the bridge ('D' / 'O')

int lastButtonReading = HIGH;
bool buttonDown = false;
unsigned long buttonChangedAt = 0;
unsigned long pressedAt = 0;
bool wasStoppedAtPress = false;
bool holdHandled = false;

bool irWasActive = false;

bool buttonPressedNow() {
  const int level = digitalRead(BUTTON_PIN);
  return BUTTON_ACTIVE_LOW ? level == LOW : level == HIGH;
}

bool irActiveNow() {
  const int level = digitalRead(IR_PIN);
  return IR_ACTIVE_HIGH ? level == HIGH : level == LOW;
}

void stopAll(const char* reason) {
  stopped = true;
  Serial.print("STOP ");
  Serial.println(reason);
}

void setup() {
  pinMode(IR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(STOP_LED_PIN, OUTPUT);
  Serial.begin(115200);
  lastButtonReading = digitalRead(BUTTON_PIN);
  Serial.println("READY");
}

void loop() {
  const unsigned long now = millis();

  // ---- button, debounced ----
  const int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) {
    buttonChangedAt = now;
    lastButtonReading = reading;
  }
  if (now - buttonChangedAt > DEBOUNCE_MS) {
    const bool down = buttonPressedNow();
    if (down && !buttonDown) {            // just pressed: always stop
      wasStoppedAtPress = stopped;
      pressedAt = now;
      holdHandled = false;
      stopAll("BUTTON");
    }
    buttonDown = down;
  }
  // Held while it was already stopped before this press: start again.
  // (A long press in a panic never restarts the robots.)
  if (buttonDown && wasStoppedAtPress && !holdHandled && now - pressedAt >= HOLD_TO_START_MS) {
    holdHandled = true;
    if (irEnabled && irArmed && irActiveNow()) {
      Serial.println("BLOCKED IR");      // someone is still moving in front of the sensor
    } else {
      stopped = false;
      irAlarm = false;
      Serial.println("START");
    }
  }

  // ---- IR sensor: stop on the start of a movement ----
  if (!irArmed && now >= IR_WARMUP_MS) {
    irArmed = true;
    irWasActive = irActiveNow();         // don't fire on a level already high at the end of warm-up
    Serial.println("IR ARMED");
  }
  const bool irActive = irActiveNow();
  if (irArmed && irEnabled && irActive && !irWasActive) {
    irAlarm = true;
    stopAll("IR");
  }
  irWasActive = irActive;

  // ---- commands from the computer ----
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == 'D') missionDone = true;
    else if (c == 'O') missionDone = false;
    else if (c == 'I') irEnabled = true;
    else if (c == 'i') { irEnabled = false; irAlarm = false; }
  }

  // ---- lights ----
  bool greenOn = false;
  if (irAlarm) {
    greenOn = (now / IR_FLASH_MS) % 2 == 0;
  } else if (missionDone) {
    greenOn = (now / DONE_BLINK_MS) % 2 == 0;
  }
  digitalWrite(LED_PIN, greenOn ? HIGH : LOW);
  digitalWrite(STOP_LED_PIN, stopped ? HIGH : LOW);
}
