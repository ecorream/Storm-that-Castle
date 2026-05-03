// libraries needed 
#include <Wire.h>
#include <FastLED.h>
#include <math.h>
#include <string.h>
#include <Servo.h>
//  SELECTOR
// Pin 40 with INPUT_PULLUP:
// Low = RED
// High =  BLUE

#define SELECTOR_PIN 40

enum TeamColor {
  TEAM_RED,
  TEAM_BLUE
};

TeamColor activeTeam = TEAM_BLUE;


// APA102 code information//
// LED 1 Selected team color
// LED 2 General system status
// LED 3  Navigation point status:
//         BLUE  = robot is navigating
//          GREEN = robot reached deployment point
//
// LED 4 IR1 detected color/ frequency display
// LED 5 IR2 detected color/ frequency display
// LED 6 IR3 detected color /frequency display

#define NUM_LEDS   6
#define DATA_PIN   51
#define CLOCK_PIN  52
#define BRIGHTNESS 50

CRGB leds[NUM_LEDS];

const uint8_t LED1_TEAM    = 0;
const uint8_t LED2_STATUS  = 1;
const uint8_t LED3_MOTION  = 2;
const uint8_t LED4_IR1     = 3;
const uint8_t LED5_IR2     = 4;
const uint8_t LED6_IR3     = 5;

// IR frecuendy sensors
// IR1  Pin 2 Left reference
// IR2  Pin 3 Center reference
// IR3 Pin 4  Right reference

#define ENABLE_IR_DISPLAY true

#define IR1_PIN 2
#define IR2_PIN 3
#define IR3_PIN 4

// this frecuencies have a upper band and a higer band
#define RED_MIN   1250.0
#define RED_MAX   1600.0

#define BLUE_MIN  700.0
#define BLUE_MAX  900.0

// Short timeout so navigation does not freeze if there is no IR signal
// 3000 us is enough for the expected 780 Hz and 1560 Hz signals
#define IR_PULSE_TIMEOUT_US 3000UL

const unsigned long IR_UPDATE_INTERVAL_MS = 200;
unsigned long lastIrUpdateTime = 0;

enum IrColor {
  IR_UNKNOWN,
  IR_RED,
  IR_BLUE
};

struct IrReading {
  float frequencyHz;
  IrColor color;
};

IrReading ir1 = {0.0, IR_UNKNOWN};
IrReading ir2 = {0.0, IR_UNKNOWN};
IrReading ir3 = {0.0, IR_UNKNOWN};

// ULTRASONIC SENSORS
// US1  left
// US2  front left
// US3  front right
// US4  right
#define US1_TRIG 30
#define US1_ECHO 31
#define US2_TRIG 32
#define US2_ECHO 33
#define US3_TRIG 34
#define US3_ECHO 35
#define US4_TRIG 36
#define US4_ECHO 37

const unsigned long ULTRASONIC_TIMEOUT_US = 12000;  // about 200 cm max range
const unsigned long SENSOR_UPDATE_INTERVAL_MS = 80;
unsigned long lastSensorUpdateTime = 0;

struct UltrasonicReading {
  float cm;
  float lastGoodCm;
  bool valid;
  bool hasLastGood;
};

UltrasonicReading us1Left   = {0, 0, false, false};
UltrasonicReading us2FrontL = {0, 0, false, false};
UltrasonicReading us3FrontR = {0, 0, false, false};
UltrasonicReading us4Right  = {0, 0, false, false};


// NAVIGATION 
//
// Sequence:
// 1) Move forward following the side wall until front distance <= FRONT_TRIGGER_CM.
// 2) Move laterally to the selected side point.
//          RED  lateral left, using US4 right sensor until RED_SIDE_ARRIVAL_CM.
//          BLUE lateral right, using US1 left sensor until BLUE_SIDE_ARRIVAL_CM.
// 3) After reaching the lateral point, move forward until DEPLOY_FRONT_CM.
// 4) Stop at deployment point. LED3 becomes GREEN.
// 
const unsigned long TOTAL_RUN_TIME_MS = 144000UL;  // 2 min and 30 s
const unsigned long LAST_ALERT_MS     = 30000UL;   //last 30 s

const float ROBOT_WIDTH_CM = 23.0;

float WALL_TARGET_CM        = 15.0;
float FRONT_TRIGGER_CM      = 20.0;
float RED_SIDE_ARRIVAL_CM   = 122.0;
float BLUE_SIDE_ARRIVAL_CM  = 83.0;
float DEPLOY_FRONT_CM       = 6.0;


// FRONT ULTRASONIC CALIBRATION
// I measured this for physical differences in the robot:
float US2_FRONT_OFFSET_CM = 0.0;
float US3_FRONT_OFFSET_CM = 1.16;

float FRONT_SAFETY_CM = 8.0;
float SIDE_SAFETY_CM  = 8.0;

float NAV_SPEED_DPS      = 560.0;
float LATERAL_SPEED_DPS  = 540.0;
float APPROACH_SPEED_DPS = 420.0;

float KP_SIDE_WALL  = 0.045;
float KP_FRONT_HOLD = 0.040;
float KP_FRONT_APP  = 0.070;

float lateralFrontReferenceCm = FRONT_TRIGGER_CM;

unsigned long robotStartTime = 0;
unsigned long stateStartTime = 0;

//SERVOS
Servo servo1;
Servo servo2;
Servo servo3;
#define SERVO1_PIN 44
#define SERVO2_PIN 49
#define SERVO3_PIN 46

//initial servo positions
const int SERVO1_INITIAL = 0;
const int SERVO2_INITIAL = 150;
const int SERVO3_INITIAL = 180;

//deployment servo positions
const int SERVO1_SECOND = 180;
const int SERVO2_SECOND = 0;
const int SERVO3_SECOND = 0;

bool servosAttached = false;


// I2C addresses miltiplexors
#define TCA_ADDR    0x70
#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE_H 0x0C


//Multiplexer channels
//D0 Front Left
//D1 Rear Left
//D3 Rear Right
//D4 Front Right

#define ENC_FRONT_LEFT   0
#define ENC_REAR_LEFT    1
#define ENC_REAR_RIGHT   3
#define ENC_FRONT_RIGHT  4

//L298 #1
//Motor A -> Front Left
//Motor B -> Front Right
#define ENA1 5
#define IN1_1 22
#define IN2_1 23
#define IN3_1 24
#define IN4_1 25
#define ENB1 6

//L298 #2
//Motor A  Rear Left
//Motor B Rear Right
#define ENA2 9
#define IN1_2 26
#define IN2_2 27
#define IN3_2 28
#define IN4_2 29
#define ENB2 10

//Deployment mechanism

#define ENA3 11
#define IN1_3 38
#define IN2_3 39

// Wheel indexes
#define IDX_FL 0
#define IDX_FR 1
#define IDX_RL 2
#define IDX_RR 3

// Direction correction
// Positive command means physical forward

bool FRONT_LEFT_REVERSED  = true;
bool FRONT_RIGHT_REVERSED = true;
bool REAR_LEFT_REVERSED   = true;
bool REAR_RIGHT_REVERSED  = true;

// Encoder-based speed control settings
float minTargetDps = 80.0;
float maxTargetDps = 1200.0;

float KP_SPEED = 0.12;
float KI_SPEED = 0.03;

int MIN_PWM = 70;
int MAX_PWM = 255;

bool STOP_MOTOR_IF_ENCODER_FAILS = true;

const unsigned long CONTROL_INTERVAL_MS = 50;
const unsigned long PRINT_INTERVAL_MS   = 500;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime   = 0;

// this measure can be change, this is going to affect navigation
float WHEEL_DIAMETER_CM = 6.5;

struct WheelController {
  const char* name;
  int enPin;
  int in1Pin;
  int in2Pin;
  uint8_t encChannel;
  bool reversed;

  int commandSign;
  float targetDps;
  float measuredSignedDps;
  float measuredAbsDps;
  float filteredDps;
  float integral;
  float pwm;

  uint16_t rawAngle;
  float angleDeg;
  uint16_t previousRaw;
  bool hasPrevious;
  bool encoderOk;

  float totalDeg;
};

WheelController wheels[4] = {
  {"FL", ENA1, IN1_1, IN2_1, ENC_FRONT_LEFT,  true, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0},
  {"FR", ENB1, IN3_1, IN4_1, ENC_FRONT_RIGHT, true, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0},
  {"RL", ENA2, IN1_2, IN2_2, ENC_REAR_LEFT,   true, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0},
  {"RR", ENB2, IN3_2, IN4_2, ENC_REAR_RIGHT,  true, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, 0}
};

const char* currentMovementName = "STOP";

// NAVIGATION STATE MACHINE
enum NavigationState {
  STATE_FORWARD_WALL_FOLLOW,
  STATE_LATERAL_SEARCH,
  STATE_APPROACH_DEPLOY,
  STATE_DEPLOY_WAIT,
  STATE_FINISHED,
  STATE_EMERGENCY_STOP
};

NavigationState navState = STATE_FORWARD_WALL_FOLLOW;

// This flag prevents the deployment code from running many times
bool deploymentActionDone = false;

// Basic Funtions helpers
float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

TeamColor readTeamFromSelector() {
  int estadoSelector = digitalRead(SELECTOR_PIN);

  if (estadoSelector == LOW) {
    return TEAM_RED;
  } else {
    return TEAM_BLUE;
  }
}

const char* teamName(TeamColor team) {
  if (team == TEAM_RED) return "ROJO";
  return "AZUL";
}

CRGB teamLedColor(TeamColor team) {
  if (team == TEAM_RED) return CRGB::Red;
  return CRGB::Blue;
}

// please be carefull this has to be activated before star the arduino
// once we start we can't change the actions
void printTeamSelector() {
  int estadoSelector = digitalRead(SELECTOR_PIN);

  Serial.print("Pin 40: ");

  if (estadoSelector == LOW) {
    Serial.println("LOW -> ROJO");
  } else {
    Serial.println("HIGH -> AZUL");
  }
}

// states
const char* stateName(NavigationState state) {
  switch (state) {
    case STATE_FORWARD_WALL_FOLLOW: return "FORWARD_WALL_FOLLOW";
    case STATE_LATERAL_SEARCH:      return "LATERAL_SEARCH";
    case STATE_APPROACH_DEPLOY:     return "APPROACH_DEPLOY";
    case STATE_DEPLOY_WAIT:         return "DEPLOY_WAIT";
    case STATE_FINISHED:            return "FINISHED";
    case STATE_EMERGENCY_STOP:      return "EMERGENCY_STOP";
    default:                        return "UNKNOWN";
  }
}

void enterState(NavigationState newState) {
  navState = newState;
  stateStartTime = millis();

  if (navState == STATE_DEPLOY_WAIT) {
    deploymentActionDone = false;
  }

  Serial.println();
  Serial.print("New navigation state: ");
  Serial.println(stateName(navState));
}

// I2C for encoders AS5600 functions multiplexor will manage the address
bool i2cDeviceFound(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return error == 0;
}

bool selectMuxChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  byte error = Wire.endTransmission();
  return error == 0;
}

bool readAS5600RawAngle(uint8_t muxChannel, uint16_t &rawAngle) {
  if (!selectMuxChannel(muxChannel)) return false;

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);

  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);

  if (Wire.available() < 2) return false;

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();

  rawAngle = ((highByte & 0x0F) << 8) | lowByte;

  return true;
}

float rawToDegrees(uint16_t rawAngle) {
  return rawAngle * 360.0 / 4096.0;
}

float angleDifference(float currentDeg, float previousDeg) {
  float diff = currentDeg - previousDeg;
  if (diff > 180.0) diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return diff;
}


// Motor functions
void driveMotorSigned(int enPin, int in1Pin, int in2Pin, int signedPwm, bool reversed) {
  signedPwm = constrain(signedPwm, -255, 255);

  if (reversed) {
    signedPwm = -signedPwm;
  }
  int pwmValue = abs(signedPwm);
  if (signedPwm > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (signedPwm < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
  analogWrite(enPin, pwmValue);
}

void stopMotor(int enPin, int in1Pin, int in2Pin) {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enPin, 0);
}

void stopAllMotors() {
  stopMotor(ENA1, IN1_1, IN2_1);
  stopMotor(ENB1, IN3_1, IN4_1);
  stopMotor(ENA2, IN1_2, IN2_2);
  stopMotor(ENB2, IN3_2, IN4_2);

  
  stopMotor(ENA3, IN1_3, IN2_3);

  for (int i = 0; i < 4; i++) {
    wheels[i].commandSign = 0;
    wheels[i].targetDps = 0;
    wheels[i].integral = 0;
    wheels[i].pwm = 0;
  }
  currentMovementName = "STOP";
}

void setWheelTarget(int index, float component, float baseSpeedDps) {
  float absComponent = fabs(component);

  if (absComponent < 0.05) {
    wheels[index].commandSign = 0;
    wheels[index].targetDps = 0;
    wheels[index].integral = 0;
    wheels[index].pwm = 0;
    return;
  }

  wheels[index].commandSign = component > 0 ? +1 : -1;
  wheels[index].targetDps = clampFloat(baseSpeedDps * absComponent, minTargetDps, maxTargetDps);
}

// x: +right, -left
// y: +forward, -backward
// rot: +rotate right, -rotate left
void setMecanumVector(float x, float y, float rot, float baseSpeedDps, const char* movementName) {
  float fl = y + x + rot;
  float fr = y - x - rot;
  float rl = y - x + rot;
  float rr = y + x - rot;

  float maxAbs = fabs(fl);

  if (fabs(fr) > maxAbs) maxAbs = fabs(fr);
  if (fabs(rl) > maxAbs) maxAbs = fabs(rl);
  if (fabs(rr) > maxAbs) maxAbs = fabs(rr);

  if (maxAbs > 1.0) {
    fl /= maxAbs;
    fr /= maxAbs;
    rl /= maxAbs;
    rr /= maxAbs;
  }

  setWheelTarget(IDX_FL, fl, baseSpeedDps);
  setWheelTarget(IDX_FR, fr, baseSpeedDps);
  setWheelTarget(IDX_RL, rl, baseSpeedDps);
  setWheelTarget(IDX_RR, rr, baseSpeedDps);

  currentMovementName = movementName;
}


// Encoder update and PI speed control
void updateWheelMeasurement(WheelController &wheel, float dtSec) {
  uint16_t currentRaw;

  if (!readAS5600RawAngle(wheel.encChannel, currentRaw)) {
    wheel.encoderOk = false;
    wheel.rawAngle = 0;
    wheel.angleDeg = 0;
    wheel.measuredSignedDps = 0;
    wheel.measuredAbsDps = 0;
    wheel.filteredDps = 0;
    return;
  }

  wheel.encoderOk = true;
  wheel.rawAngle = currentRaw;
  wheel.angleDeg = rawToDegrees(currentRaw);

  if (!wheel.hasPrevious) {
    wheel.previousRaw = currentRaw;
    wheel.hasPrevious = true;
    wheel.measuredSignedDps = 0;
    wheel.measuredAbsDps = 0;
    wheel.filteredDps = 0;
    return;
  }

  float currentDeg = rawToDegrees(currentRaw);
  float previousDeg = rawToDegrees(wheel.previousRaw);
  float diffDeg = angleDifference(currentDeg, previousDeg);

  wheel.measuredSignedDps = diffDeg / dtSec;
  wheel.measuredAbsDps = fabs(wheel.measuredSignedDps);
  wheel.filteredDps = 0.70 * wheel.filteredDps + 0.30 * wheel.measuredAbsDps;
  wheel.totalDeg += diffDeg;

  wheel.previousRaw = currentRaw;
}

void updateWheelControl(WheelController &wheel, float dtSec) {
  if (wheel.commandSign == 0 || wheel.targetDps <= 0) {
    wheel.integral = 0;
    wheel.pwm = 0;
    driveMotorSigned(wheel.enPin, wheel.in1Pin, wheel.in2Pin, 0, wheel.reversed);
    return;
  }

  if (!wheel.encoderOk && STOP_MOTOR_IF_ENCODER_FAILS) {
    wheel.integral = 0;
    wheel.pwm = 0;
    driveMotorSigned(wheel.enPin, wheel.in1Pin, wheel.in2Pin, 0, wheel.reversed);
    return;
  }

  float basePwm = wheel.targetDps * 0.32;
  basePwm = clampFloat(basePwm, MIN_PWM, MAX_PWM);
  float error = wheel.targetDps - wheel.filteredDps;
  wheel.integral += error * dtSec;
  wheel.integral = clampFloat(wheel.integral, -600.0, 600.0);
  float outputPwm = basePwm + KP_SPEED * error + KI_SPEED * wheel.integral;
  outputPwm = clampFloat(outputPwm, MIN_PWM, MAX_PWM);

  wheel.pwm = outputPwm;

  int signedPwm = wheel.commandSign * (int)wheel.pwm;

  driveMotorSigned(wheel.enPin, wheel.in1Pin, wheel.in2Pin, signedPwm, wheel.reversed);
}

void updateAllWheelControllers() {
  unsigned long now = millis();
  if (now - lastControlTime < CONTROL_INTERVAL_MS) return;
  float dtSec = (now - lastControlTime) / 1000.0;
  lastControlTime = now;

  if (dtSec <= 0) return;

  for (int i = 0; i < 4; i++) {
    updateWheelMeasurement(wheels[i], dtSec);
  }

  for (int i = 0; i < 4; i++) {
    updateWheelControl(wheels[i], dtSec);
  }
}

// Ultrasonic 

void setupUltrasonicPins() {
  pinMode(US1_TRIG, OUTPUT);
  pinMode(US1_ECHO, INPUT);

  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);

  pinMode(US3_TRIG, OUTPUT);
  pinMode(US3_ECHO, INPUT);

  pinMode(US4_TRIG, OUTPUT);
  pinMode(US4_ECHO, INPUT);

  digitalWrite(US1_TRIG, LOW);
  digitalWrite(US2_TRIG, LOW);
  digitalWrite(US3_TRIG, LOW);
  digitalWrite(US4_TRIG, LOW);
}

float readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);

  if (duration == 0) {
    return -1.0;
  }

  float cm = duration * 0.0343 / 2.0;

  if (cm < 2.0 || cm > 200.0) {
    return -1.0;
  }

  return cm;
}

void updateOneUltrasonic(UltrasonicReading &reading, int trigPin, int echoPin) {
  float value = readUltrasonicCm(trigPin, echoPin);

  if (value > 0) {
    reading.cm = value;
    reading.lastGoodCm = value;
    reading.valid = true;
    reading.hasLastGood = true;
  } else {
    reading.valid = false;

    if (reading.hasLastGood) {
      reading.cm = reading.lastGoodCm;
    } else {
      reading.cm = -1.0;
    }
  }
}

void updateUltrasonicSensors() {
  unsigned long now = millis();

  if (now - lastSensorUpdateTime < SENSOR_UPDATE_INTERVAL_MS) return;

  lastSensorUpdateTime = now;

  updateOneUltrasonic(us1Left, US1_TRIG, US1_ECHO);
  delayMicroseconds(500);
  updateOneUltrasonic(us2FrontL, US2_TRIG, US2_ECHO);
  delayMicroseconds(500);
  updateOneUltrasonic(us3FrontR, US3_TRIG, US3_ECHO);
  delayMicroseconds(500);

  updateOneUltrasonic(us4Right, US4_TRIG, US4_ECHO);
}

float correctedUS2FrontCm() {
  return us2FrontL.cm + US2_FRONT_OFFSET_CM;
}

float correctedUS3FrontCm() {
  return us3FrontR.cm + US3_FRONT_OFFSET_CM;
}

float correctedUS2FrontLastGoodCm() {
  return us2FrontL.lastGoodCm + US2_FRONT_OFFSET_CM;
}

float correctedUS3FrontLastGoodCm() {
  return us3FrontR.lastGoodCm + US3_FRONT_OFFSET_CM;
}

bool getFrontAverage(float &avgCm, bool requireFreshReading) {
  float sum = 0;
  int count = 0;

  if (us2FrontL.valid) {
    sum += correctedUS2FrontCm();
    count++;
  }

  if (us3FrontR.valid) {
    sum += correctedUS3FrontCm();
    count++;
  }

  if (count > 0) {
    avgCm = sum / count;
    return true;
  }

  if (!requireFreshReading) {
    if (us2FrontL.hasLastGood && us3FrontR.hasLastGood) {
      avgCm = (correctedUS2FrontLastGoodCm() + correctedUS3FrontLastGoodCm()) / 2.0;
      return true;
    }

    if (us2FrontL.hasLastGood) {
      avgCm = correctedUS2FrontLastGoodCm();
      return true;
    }

    if (us3FrontR.hasLastGood) {
      avgCm = correctedUS3FrontLastGoodCm();
      return true;
    }
  }

  avgCm = -1.0;
  return false;
}

bool getSideDistanceForTeam(TeamColor team, float &sideCm) {
  if (team == TEAM_RED) {
    if (us4Right.valid) {
      sideCm = us4Right.cm;
      return true;
    }

    if (us4Right.hasLastGood) {
      sideCm = us4Right.lastGoodCm;
      return true;
    }
  } else {
    if (us1Left.valid) {
      sideCm = us1Left.cm;
      return true;
    }

    if (us1Left.hasLastGood) {
      sideCm = us1Left.lastGoodCm;
      return true;
    }
  }

  sideCm = -1.0;
  return false;
}

// IR FREQUENCY fucntions
float readFrequency(int pin) {
  unsigned long highTime = pulseIn(pin, HIGH, IR_PULSE_TIMEOUT_US);
  unsigned long lowTime  = pulseIn(pin, LOW, IR_PULSE_TIMEOUT_US);

  if (highTime == 0 || lowTime == 0) {
    return 0.0;
  }

  unsigned long period = highTime + lowTime;

  if (period == 0) {
    return 0.0;
  }

  float frequency = 1000000.0 / period;

  return frequency;
}

IrColor detectIrColor(float frequency) {
  if (frequency >= RED_MIN && frequency <= RED_MAX) {
    return IR_RED;
  } else if (frequency >= BLUE_MIN && frequency <= BLUE_MAX) {
    return IR_BLUE;
  } else {
    return IR_UNKNOWN;
  }
}

const char* irColorName(IrColor color) {
  switch (color) {
    case IR_RED:
      return "RED";

    case IR_BLUE:
      return "BLUE";

    default:
      return "UNKNOWN";
  }
}

CRGB irColorToLed(IrColor color) {
  switch (color) {
    case IR_RED:
      return CRGB::Red;

    case IR_BLUE:
      return CRGB::Blue;

    default:
      return CRGB::Black;
  }
}

void updateIrDisplay() {
  if (!ENABLE_IR_DISPLAY) {
    ir1.frequencyHz = 0.0;
    ir2.frequencyHz = 0.0;
    ir3.frequencyHz = 0.0;

    ir1.color = IR_UNKNOWN;
    ir2.color = IR_UNKNOWN;
    ir3.color = IR_UNKNOWN;

    return;
  }

  unsigned long now = millis();

  if (now - lastIrUpdateTime < IR_UPDATE_INTERVAL_MS) return;

  lastIrUpdateTime = now;

  ir1.frequencyHz = readFrequency(IR1_PIN);
  ir2.frequencyHz = readFrequency(IR2_PIN);
  ir3.frequencyHz = readFrequency(IR3_PIN);

  ir1.color = detectIrColor(ir1.frequencyHz);
  ir2.color = detectIrColor(ir2.frequencyHz);
  ir3.color = detectIrColor(ir3.frequencyHz);
}

void printOneIrSensor(const char* label, IrReading reading, int colorWidth) {
  Serial.print(label);
  Serial.print(": ");

  const char* colorText = irColorName(reading.color);
  Serial.print(colorText);

  int len = strlen(colorText);

  for (int i = len; i < colorWidth; i++) {
    Serial.print(" ");
  }

  Serial.print(" ");
  Serial.print(reading.frequencyHz, 1);
  Serial.print(" Hz");
}

void printIrData() {
  printOneIrSensor("IR1", ir1, 8);
  Serial.print(" | ");

  printOneIrSensor("IR2", ir2, 8);
  Serial.print(" | ");

  printOneIrSensor("IR3", ir3, 8);
  Serial.println();
}


// LED state update, this shows several procces during the operation
void updateLeds() {
  unsigned long now = millis();
  unsigned long elapsed = now - robotStartTime;

  bool timeFinished = elapsed >= TOTAL_RUN_TIME_MS;
  bool blink = ((now / 300) % 2) == 0;

  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // LED1: selected team
  leds[LED1_TEAM] = teamLedColor(activeTeam);

  // LED2: general status
  if (timeFinished || navState == STATE_FINISHED) {
    leds[LED2_STATUS] = CRGB::Yellow;
  } else {
    leds[LED2_STATUS] = blink ? CRGB::Yellow : CRGB::Black;
  }

  // LED3:
  // BLUE=navigating
  // GREEN=reached deployment point
  if (navState == STATE_FORWARD_WALL_FOLLOW ||
      navState == STATE_LATERAL_SEARCH ||
      navState == STATE_APPROACH_DEPLOY) {

    leds[LED3_MOTION] = CRGB::Blue;

  } else if (navState == STATE_DEPLOY_WAIT) {

    leds[LED3_MOTION] = CRGB::Green;

  } else {

    leds[LED3_MOTION] = CRGB::Black;
  }

  leds[LED4_IR1] = irColorToLed(ir1.color);
  leds[LED5_IR2] = irColorToLed(ir2.color);
  leds[LED6_IR3] = irColorToLed(ir3.color);

  FastLED.show();
}


// Safety and navigation, this is just in case of the robot is too close, we can change this later
float sideSafetyXCorrection(float x) {
  if (us1Left.valid && us1Left.cm < SIDE_SAFETY_CM) {
    x = 0.50;  // too close to left wall, move right
  }

  if (us4Right.valid && us4Right.cm < SIDE_SAFETY_CM) {
    x = -0.50; // too close to right wall, move left
  }

  return x;
}

bool frontEmergencyDetected() {
  float frontCm;
  if (!getFrontAverage(frontCm, true)) {
    return false;
  }

  return frontCm <= FRONT_SAFETY_CM;
}

void navigateForwardWallFollow() {
  float frontCm;
  bool frontFresh = getFrontAverage(frontCm, true);

  // Step 1 complete:
  // Front wall found. Stop forward motion and start lateral search.
  if (frontFresh && frontCm <= FRONT_TRIGGER_CM) {
    lateralFrontReferenceCm = frontCm;
    stopAllMotors();
    enterState(STATE_LATERAL_SEARCH);
    return;
  }

  float x = 0.0;
  float y = 1.0;
  float rot = 0.0;  // Parallel correction is intentionally not used (this was the first )

  float sideCm;

  if (getSideDistanceForTeam(activeTeam, sideCm)) {
    float error = sideCm - WALL_TARGET_CM;

    if (activeTeam == TEAM_RED) {
      // RED uses right wall. If too far from the right wall, move right
      x = KP_SIDE_WALL * error;
    } else {
      // BLUE uses left wall. If too far from the left wall, move left
      x = -KP_SIDE_WALL * error;
    }

    x = clampFloat(x, -0.35, 0.35);
  }

  x = sideSafetyXCorrection(x);

  if (frontEmergencyDetected()) {
    stopAllMotors();
    enterState(STATE_EMERGENCY_STOP);
    return;
  }

  setMecanumVector(x, y, rot, NAV_SPEED_DPS, "FORWARD WALL FOLLOW");
}

void attachServosAtInitial(){

  servo1.write(SERVO1_INITIAL);
  servo2.write(SERVO2_INITIAL);
  servo3.write(SERVO3_INITIAL);

  if (!servosAttached) {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servosAttached = true;
  }

  delay(300);

  servo1.write(SERVO1_INITIAL);
  servo2.write(SERVO2_INITIAL);
  servo3.write(SERVO3_INITIAL);

}
/*
we can use this or not if we want more presicion 
void fineCalibrationLateral(){
  float sideCm;
  bool sideAvailable = getSideDistanceForTeam(activeTeam, sideCm);
  float arrivalCm = activeTeam == TEAM_RED ? RED_SIDE_ARRIVAL_CM : BLUE_SIDE_ARRIVAL_CM;
  if(sideAvailable && abs(sideCm - arrivalCm) > 0.5){
  }
}*/

void navigateLateralSearch() {
  float frontCm;
  bool frontAvailable = getFrontAverage(frontCm, false);

  float sideCm;
  bool sideAvailable = getSideDistanceForTeam(activeTeam, sideCm);

  float arrivalCm = activeTeam == TEAM_RED ? RED_SIDE_ARRIVAL_CM : BLUE_SIDE_ARRIVAL_CM;

  // Step 2 complete:
  // The robot reached the lateral point. Now it will approach forward
  if (sideAvailable && sideCm >= arrivalCm) {
    stopAllMotors();
    enterState(STATE_APPROACH_DEPLOY);
    return;
  }

  // RED moves left and when team BLUE moves right
  float x = activeTeam == TEAM_RED ? -1.0 : 1.0;
  float y = 0.0;
  float rot = 0.0;  // Parallel correction is intentionally not used

  // Keep the approximate front distance while moving laterally
  // This uses only the front average, not a parallel check
  if (frontAvailable) {
    float errorFront = frontCm - lateralFrontReferenceCm;
    y = KP_FRONT_HOLD * errorFront;
    y = clampFloat(y, -0.30, 0.30);
  }

  x = sideSafetyXCorrection(x);

  setMecanumVector(x, y, rot, LATERAL_SPEED_DPS, "LATERAL SEARCH");
}

void navigateApproachDeploy() {
  float frontCm;

  if (!getFrontAverage(frontCm, true)) {
    stopAllMotors();
    currentMovementName = "WAITING FRONT SENSOR";
    return;
  }

  float error = frontCm - DEPLOY_FRONT_CM;
  float y = 0.0;
  float x = 0.0;
  float rot = 0.0;  // Parallel correction is intentionally not used

  // Step 3 complete:
  // Once the robot reaches the desired front distance, stop
  // No parallel condition is required
  if (frontCm <= DEPLOY_FRONT_CM) {
    stopAllMotors();
    enterState(STATE_DEPLOY_WAIT);
    return;
  }

  if (frontCm <= FRONT_SAFETY_CM) {
    y = -0.30;
  } else if (error > 8.0) {
    y = clampFloat(KP_FRONT_APP * error, 0.35, 0.70);
  } else if (error > 1.0) {
    y = clampFloat(KP_FRONT_APP * error, 0.20, 0.45);
  } else if (error < -0.8) {
    y = -0.20;
  } else {
    y = 0.0;
  }

  x = sideSafetyXCorrection(x);

  setMecanumVector(x, y, rot, APPROACH_SPEED_DPS, "APPROACH DEPLOY");
}

void handleEmergencyStop() {
  stopAllMotors();

  float frontCm;
  bool frontFresh = getFrontAverage(frontCm, true);

  // Recover only when the front wall is no longer dangerously close
  if (!frontFresh || frontCm > (FRONT_SAFETY_CM + 3.0)) {
    enterState(STATE_FORWARD_WALL_FOLLOW);
  }
}
// DEPLOYMENT 
// This function runs once when the robot reaches the point

void runDeploymentAction() {
  Serial.println();
  Serial.println("ROBOT REACHED DEPLOYMENT POINT");
  Serial.println("Deployment placeholder running.");
  Serial.println("Add your deployment code inside runDeploymentAction().");
 
  driveMotorSigned(ENA3, IN1_3, IN2_3, 255, false);
  // tis tina can be change
  delay(6900);
  stopMotor(ENA3, IN1_3, IN2_3);
  Serial.print("motor stoped");
  
  Serial.print("Servo activated");
  // this code can cange after, this is going to give us the last 10-5 seconds before the end of the round
  delay(95000);

       TeamColor currentTeam = readTeamFromSelector();
       IrColor left_color = detectIrColor(ir1.frequencyHz);
       IrColor mid_color = detectIrColor(ir2.frequencyHz);
       IrColor right_color = detectIrColor(ir3.frequencyHz);  


       // the logic here will give is that if we detect the opposit loyalty we score
       // also if due to the error in the location or malfuntion we are going to try
       // this will increase our opportunity to get the round

       if(currentTeam == TEAM_RED){
         //deploy
         if(left_color == IR_RED || left_color == IR_UNKNOWN){
           servo1.write(SERVO1_SECOND);          
         }
         if(mid_color == IR_RED || mid_color == IR_UNKNOWN){
           servo2.write(SERVO2_SECOND);
         }
         if(right_color == IR_RED || right_color == IR_UNKNOWN){
           servo3.write(SERVO3_SECOND);
         }
       }
       else{
//         //deploy
         if(left_color == IR_BLUE || left_color == IR_UNKNOWN){
           servo1.write(SERVO1_SECOND);          
         }
         if(mid_color == IR_BLUE || mid_color == IR_UNKNOWN){
           servo2.write(SERVO2_SECOND);
         }
         if(right_color == IR_BLUE || right_color == IR_UNKNOWN){
           servo3.write(SERVO3_SECOND);
         }
       }
  delay(2000);
  Serial.print("finished");
//reset 
  servo1.write(SERVO1_INITIAL);
  servo2.write(SERVO2_INITIAL);
  servo3.write(SERVO3_INITIAL);
}

void handleDeployWaitState() {
  stopAllMotors();

  currentMovementName = "AT DEPLOYMENT POINT";

  // Run deployment action only one time
  if (!deploymentActionDone) {
    deploymentActionDone = true;
    runDeploymentAction();
  }
}

void updateNavigation() {
  unsigned long elapsed = millis() - robotStartTime;

  if (elapsed >= TOTAL_RUN_TIME_MS) {
    if (navState != STATE_FINISHED) {
      stopAllMotors();
      enterState(STATE_FINISHED);
    }

    return;
  }

  //these are the actions 
  switch (navState) {
    case STATE_FORWARD_WALL_FOLLOW:
      navigateForwardWallFollow();
      break;
    case STATE_LATERAL_SEARCH:
      navigateLateralSearch();
      break;
    case STATE_APPROACH_DEPLOY:
      navigateApproachDeploy();
      break;
    case STATE_DEPLOY_WAIT:
      handleDeployWaitState();
      break;
    case STATE_EMERGENCY_STOP:
      handleEmergencyStop();
      break;
    case STATE_FINISHED:
      stopAllMotors();
      break;
  }
}

// Diagnostics wheels for control, also this code has prints for debugging and detect if is there any problem with the encoders
// mostly this is for harware detections

float wheelDistanceCm(WheelController &wheel) {
  float circumference = PI * WHEEL_DIAMETER_CM;
  return (wheel.totalDeg / 360.0) * circumference;
}

void printWheelData() {
  Serial.println();

  Serial.print("Movement: ");
  Serial.print(currentMovementName);

  Serial.print(" | State: ");
  Serial.print(stateName(navState));

  Serial.print(" | Team: ");
  Serial.println(teamName(activeTeam));
  // just for debuggin purposes 
  Serial.println("Wheel | Encoder | Raw  | Angle | Speed | Filtered | Target | PWM | Dist cm | Status");

  int order[4] = {IDX_FL, IDX_RL, IDX_RR, IDX_FR};

  for (int j = 0; j < 4; j++) {
    int i = order[j];

    Serial.print(wheels[i].name);
    Serial.print("    | D");
    Serial.print(wheels[i].encChannel);
    Serial.print("      | ");
    Serial.print(wheels[i].rawAngle);
    Serial.print(" | ");
    Serial.print(wheels[i].angleDeg, 1);
    Serial.print(" | ");
    Serial.print(wheels[i].measuredAbsDps, 1);
    Serial.print(" | ");
    Serial.print(wheels[i].filteredDps, 1);
    Serial.print(" | ");
    Serial.print(wheels[i].targetDps, 1);
    Serial.print(" | ");
    Serial.print(wheels[i].pwm, 0);
    Serial.print(" | ");
    Serial.print(wheelDistanceCm(wheels[i]), 1);
    Serial.print(" | ");
    if (wheels[i].encoderOk) {
      Serial.println("OK");
    } else {
      Serial.println("ENCODER ERROR");
    }
  }
}

void printSensorData() {
  float frontCm;
  bool frontFresh = getFrontAverage(frontCm, true);

  Serial.print("US1 Left: ");

  if (us1Left.valid) Serial.print(us1Left.cm, 1);
  else Serial.print("NO FRESH");

  Serial.print(" cm | US2 raw: ");

  if (us2FrontL.valid) Serial.print(us2FrontL.cm, 1);
  else Serial.print("NO FRESH");

  Serial.print(" cm | US2 corr: ");

  if (us2FrontL.valid) Serial.print(correctedUS2FrontCm(), 1);
  else Serial.print("NO FRESH");

  Serial.print(" cm | US3 raw: ");

  if (us3FrontR.valid) Serial.print(us3FrontR.cm, 1);
  else Serial.print("NO FRESH");

  Serial.print(" cm | US3 corr: ");

  if (us3FrontR.valid) Serial.print(correctedUS3FrontCm(), 1);
  else Serial.print("NO FRESH");

  Serial.print(" cm | US4 Right: ");

  if (us4Right.valid) Serial.print(us4Right.cm, 1);
  else Serial.print("NO FRESH");

  Serial.print(" cm | Front avg corrected: ");

  if (frontFresh) Serial.print(frontCm, 1);
  else Serial.print("NO FRESH");

  Serial.println();
}

void printI2CDiagnostic() {
  Serial.println();
  Serial.println("I2C DIAGNOSTIC ");

  Serial.print("Checking TCA9548A multiplexer at address 0x70: ");

  if (i2cDeviceFound(TCA_ADDR)) {
    Serial.println("FOUND");
  } else {
    Serial.println("NOT FOUND");
    Serial.println("Check SDA=20, SCL=21, VCC, and GND.");
    Serial.println("The program will continue, but encoders will show errors.");
    Serial.println("----");
    return;
  }

  Serial.println("Checking AS5600 encoders by multiplexer channel:");
  for (int channel = 0; channel < 8; channel++) {
    bool muxOk = selectMuxChannel(channel);

    Serial.print("Channel D");
    Serial.print(channel);
    Serial.print(": ");

    if (!muxOk) {
      Serial.println("MUX SELECT ERROR");
      continue;
    }

    if (i2cDeviceFound(AS5600_ADDR)) {
      Serial.println("AS5600 FOUND");
    } else {
      Serial.println("No AS5600");
    }
  }
  Serial.println("------");
}

void initializeEncoders() {
  Serial.println();
  Serial.println("Initializing encoder values...");

  for (int i = 0; i < 4; i++) {
    uint16_t raw;

    wheels[i].reversed = true;

    if (i == IDX_FL) wheels[i].reversed = FRONT_LEFT_REVERSED;
    if (i == IDX_FR) wheels[i].reversed = FRONT_RIGHT_REVERSED;
    if (i == IDX_RL) wheels[i].reversed = REAR_LEFT_REVERSED;
    if (i == IDX_RR) wheels[i].reversed = REAR_RIGHT_REVERSED;

    Serial.print("Reading ");
    Serial.print(wheels[i].name);
    Serial.print(" on D");
    Serial.print(wheels[i].encChannel);
    Serial.print(": ");

    if (readAS5600RawAngle(wheels[i].encChannel, raw)) {
      wheels[i].rawAngle = raw;
      wheels[i].angleDeg = rawToDegrees(raw);
      wheels[i].previousRaw = raw;
      wheels[i].hasPrevious = true;
      wheels[i].encoderOk = true;
      wheels[i].totalDeg = 0;

      Serial.print("OK | Raw = ");
      Serial.print(raw);
      Serial.print(" | Angle = ");
      Serial.println(wheels[i].angleDeg, 1);
    } else {
      wheels[i].rawAngle = 0;
      wheels[i].angleDeg = 0;
      wheels[i].previousRaw = 0;
      wheels[i].hasPrevious = false;
      wheels[i].encoderOk = false;
      wheels[i].totalDeg = 0;

      Serial.println("ERROR");
    }
  }

  Serial.println("Encoder initialization finished.");
}

void setupPins() {
  pinMode(ENA1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  pinMode(IN3_1, OUTPUT);
  pinMode(IN4_1, OUTPUT);
  pinMode(ENB1, OUTPUT);

  pinMode(ENA2, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  pinMode(IN3_2, OUTPUT);
  pinMode(IN4_2, OUTPUT);
  pinMode(ENB2, OUTPUT);

  pinMode(ENA3, OUTPUT);
  pinMode(IN1_3, OUTPUT);
  pinMode(IN2_3, OUTPUT);

  pinMode(SELECTOR_PIN, INPUT_PULLUP);

  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
}

void printStartupSummary() {

// just for debuging 
  Serial.println();
  Serial.println("Mecanum autonomous navigation with encoders + ultrasonic sensors + IR frequency display");
  Serial.println("Selector: pin 40 LOW=ROJO, HIGH=AZUL");
  Serial.println("Sequence: FORWARD -> LATERAL POINT -> APPROACH FRONT DISTANCE -> DEPLOYMENT POINT");
  Serial.println("Parallel check: DISABLED");
  Serial.println("RED route: US4 right wall + lateral left + US4 side arrival");
  Serial.println("BLUE route: US1 left wall + lateral right + US1 side arrival");
  Serial.println("LED3 BLUE = navigating");
  Serial.println("LED3 GREEN = deployment point reached");
  Serial.println("LED4 = IR1 frequency display");
  Serial.println("LED5 = IR2 frequency display");
  Serial.println("LED6 = IR3 frequency display");

  printTeamSelector();

  Serial.print("Active team locked at startup: ");
  Serial.println(teamName(activeTeam));

  Serial.println();
  Serial.println("Main variables:");

  Serial.print("WALL_TARGET_CM = ");
  Serial.println(WALL_TARGET_CM);

  Serial.print("FRONT_TRIGGER_CM = ");
  Serial.println(FRONT_TRIGGER_CM);

  Serial.print("RED_SIDE_ARRIVAL_CM = ");
  Serial.println(RED_SIDE_ARRIVAL_CM);

  Serial.print("BLUE_SIDE_ARRIVAL_CM = ");
  Serial.println(BLUE_SIDE_ARRIVAL_CM);

  Serial.print("DEPLOY_FRONT_CM = ");
  Serial.println(DEPLOY_FRONT_CM);

  Serial.print("US2_FRONT_OFFSET_CM = ");
  Serial.println(US2_FRONT_OFFSET_CM);

  Serial.print("US3_FRONT_OFFSET_CM = ");
  Serial.println(US3_FRONT_OFFSET_CM);

  Serial.print("TOTAL_RUN_TIME_MS = ");
  Serial.println(TOTAL_RUN_TIME_MS);

  Serial.println();
  Serial.println("IR frequency ranges:");

  Serial.print("BLUE: ");
  Serial.print(BLUE_MIN);
  Serial.print(" Hz to ");
  Serial.print(BLUE_MAX);
  Serial.println(" Hz");

  Serial.print("RED: ");
  Serial.print(RED_MIN);
  Serial.print(" Hz to ");
  Serial.print(RED_MAX);
  Serial.println(" Hz");
}
// Setup
void setup() {
  Serial.begin(9600);
  delay(1500);

  setupPins();
  setupUltrasonicPins();

  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  activeTeam = readTeamFromSelector();

  printStartupSummary();

  Wire.begin();  //Arduino Mega: SDA = 20, SCL = 21 this is for the multiplexor
  Wire.setWireTimeout(3000, true);

  attachServosAtInitial();
  stopAllMotors();

  printI2CDiagnostic();
  initializeEncoders();

  robotStartTime = millis();
  stateStartTime = millis();

  lastControlTime = millis();
  lastPrintTime = millis();

  lastSensorUpdateTime = 0;
  lastIrUpdateTime = 0;

  enterState(STATE_FORWARD_WALL_FOLLOW);
}

// Main loop, the logic will be developed in this part
void loop() {
  updateAllWheelControllers();

  updateUltrasonicSensors();
  // This always reads the IR frequency and updates LED4, LED5, LED6.
  updateIrDisplay();
  updateNavigation();
  updateLeds();
  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = millis();
    Serial.println();
    printTeamSelector();
    Serial.print("Time elapsed: ");
    Serial.print((millis() - robotStartTime) / 1000.0, 1);

    Serial.print(" s | State: ");
    Serial.print(stateName(navState));

    Serial.print(" | Movement: ");
    Serial.println(currentMovementName);

    printIrData();
    printSensorData();
    printWheelData();
  }
}
