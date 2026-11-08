#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "soc/gpio_struct.h"

// Arduino generates function prototypes near the top of .ino files.
// Forward-declare the motor type so its generated prototypes can use it.
struct StepperState;

// ================================================================
// ESP32 STEPPER BALANCE BOT
// BalancingWii-style control
// + step-count position tracking
// + fixed return speed
// + stronger return integral
// + integral bleed only near home
// ================================================================


// ================================================================
// PINS
// ================================================================

const int LEFT_STEP_PIN  = 25;
const int LEFT_DIR_PIN   = 26;

const int RIGHT_STEP_PIN = 19;
const int RIGHT_DIR_PIN  = 23;

const int ENABLE_PIN = 13;

const int ARM_BUTTON_PIN = 32;
const int TRIM_POT_PIN   = 34;

const int SDA_PIN = 21;
const int SCL_PIN = 22;


// ================================================================
// STEPPER SETTINGS
// ================================================================

const int FULL_STEPS_PER_REV = 200;
const int MICROSTEPS = 16;

const int STEPS_PER_REV =
  FULL_STEPS_PER_REV * MICROSTEPS;   // 1600

const int LEFT_MOTOR_SIGN  = -1;
const int RIGHT_MOTOR_SIGN = +1;


// ================================================================
// CONTROL LOOP
// ================================================================

const uint32_t CONTROL_PERIOD_US = 5000;
const float DT = 0.005f;


// ================================================================
// ANGLE CONTROL
// ================================================================

float ANGLE_KP = 120.0f;
float ANGLE_KD = 4.0f;


// ================================================================
// SPEED LOOP
// ================================================================

float SPEED_KP = 0.02f;


// ================================================================
// RETURN CONTROL
// ================================================================

const float RETURN_RPM = 5.0f;

float RETURN_KI = 0.04f;

// Allow enough integral authority to overcome trim bias.
const float MAX_RETURN_I_ANGLE = 1.0f;

// Start dumping integral once inside 1 wheel revolution.
const int32_t RETURN_SLOW_STEPS =
  1L * STEPS_PER_REV;              // 1600

// Integral bleed rate near home.
const float RETURN_I_NEAR_HOME_DECAY = 0.98f;

float returnIntegral = 0.0f;


// ================================================================
// POSITION RETURN LIMITS
// ================================================================

// Start return after 2 wheel revolutions.
const int32_t RETURN_START_STEPS =
  2L * STEPS_PER_REV;              // 3200

// Stop return inside 1/2 wheel revolution.
const int32_t RETURN_STOP_STEPS =
  STEPS_PER_REV / 2;               // 800

int returnDirection = 0;


// ================================================================
// TARGET ANGLE LIMIT
// ================================================================

const float MAX_TARGET_ANGLE = 12.0f;


// ================================================================
// MOTOR SPEED
// ================================================================

const float MAX_DESIRED_RPM = 160.0f;

const float MOTOR_RPM_SLEW_PER_SEC = 800.0f;

const float MOTOR_RPM_SLEW_PER_LOOP =
  MOTOR_RPM_SLEW_PER_SEC * DT;

const float MIN_ACTIVE_RPM = 1.0f;


// ================================================================
// SPEED FILTER
// ================================================================

const float SPEED_FILTER_OLD = 0.92f;
const float SPEED_FILTER_NEW = 0.08f;

// ESP-NOW joystick (matches HW675_Joystick_Remote packet exactly).
// Keep the bot's physical button as its balance ARM/DISARM control.
const uint16_t REMOTE_MAGIC = 0xB017;
const uint32_t REMOTE_TIMEOUT_MS = 300;
const float REMOTE_MAX_SPEED_RPM = 75.0f;  // higher forward/reverse top speed
const float REMOTE_MAX_TURN_RPM = 12.0f;
const float REMOTE_SPEED_RAMP_RPM_S = 120.0f;
const float REMOTE_TURN_RAMP_RPM_S = 60.0f;
const float REMOTE_SPEED_KP = 0.08f; // 4x drive authority; return keeps 0.02
const float REMOTE_MAX_TILT_DEG = 7.0f;

struct __attribute__((packed)) ControlPacket {
  uint16_t signature;
  uint16_t sequence;
  int16_t throttle;
  int16_t steering;
  uint8_t armed;
};

portMUX_TYPE remoteMux = portMUX_INITIALIZER_UNLOCKED;
ControlPacket latestRemote = {};
uint32_t latestRemoteAt = 0;
bool receivedRemote = false;
float remoteSpeedRPM = 0.0f;
float remoteTurnRPM = 0.0f;
bool remoteWasMoving = false;

void onRemotePacket(const esp_now_recv_info_t *info,
                    const uint8_t *data, int length) {
  if (length != sizeof(ControlPacket)) return;
  ControlPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.signature != REMOTE_MAGIC || p.armed > 1 ||
      abs((int)p.throttle) > 1000 || abs((int)p.steering) > 1000) return;
  portENTER_CRITICAL(&remoteMux);
  latestRemote = p;
  latestRemoteAt = millis();
  receivedRemote = true;
  portEXIT_CRITICAL(&remoteMux);
}

float approachValue(float current, float target, float maxChange) {
  return current + constrain(target - current, -maxChange, maxChange);
}


// ================================================================
// BMI160
// ================================================================

const uint8_t BMI_ADDR = 0x68;

const float GYRO_SCALE = 65.6f;

const float GYRO_SIGN  = -1.0f;
const float ACCEL_SIGN = +1.0f;

const float FILTER_GYRO = 0.995f;


// ================================================================
// TRIM
// ================================================================

const float TRIM_RANGE_DEG = 4.0f;

int potZero = 2048;
float trimAngle = 0.0f;


// ================================================================
// SAFETY
// ================================================================

const float FALL_ANGLE = 35.0f;


// ================================================================
// STATE
// ================================================================

bool armed = false;

float gyroZero  = 0.0f;
float accelZero = 0.0f;

float accelAngle = 0.0f;
float angleDeg   = 0.0f;
float gyroRate   = 0.0f;

float desiredMotorRPM = 0.0f;
float actualMotorRPM  = 0.0f;

float filteredMotorRPM = 0.0f;


// ================================================================
// POSITION STATE
// ================================================================

volatile int32_t robotStepPosition = 0;

volatile int8_t leftPositionStepSign = 0;


// ================================================================
// FIXED 40 kHz DDS TIMER
// ================================================================

const uint32_t STEP_TIMER_BASE_HZ = 1000000;
const uint32_t STEP_TIMER_TICKS   = 25;

const float STEP_ISR_HZ =
  (float)STEP_TIMER_BASE_HZ /
  STEP_TIMER_TICKS;

hw_timer_t *stepTimer = NULL;

const uint32_t LEFT_STEP_MASK =
  (1UL << LEFT_STEP_PIN);

const uint32_t RIGHT_STEP_MASK =
  (1UL << RIGHT_STEP_PIN);

volatile uint32_t leftPhase  = 0;
volatile uint32_t rightPhase = 0;

volatile uint32_t leftPhaseIncrement  = 0;
volatile uint32_t rightPhaseIncrement = 0;

volatile bool leftPulseHigh  = false;
volatile bool rightPulseHigh = false;

portMUX_TYPE stepMux =
  portMUX_INITIALIZER_UNLOCKED;


// ================================================================
// MOTOR STATE
// ================================================================

struct StepperState
{
  int dirPin;
  int motorSign;
  int lastLogicalDirection;
};

StepperState leftMotor =
{
  LEFT_DIR_PIN,
  LEFT_MOTOR_SIGN,
  0
};

StepperState rightMotor =
{
  RIGHT_DIR_PIN,
  RIGHT_MOTOR_SIGN,
  0
};


// ================================================================
// FAST GPIO
// ================================================================

inline void IRAM_ATTR leftStepHigh()
{
  GPIO.out_w1ts = LEFT_STEP_MASK;
}

inline void IRAM_ATTR leftStepLow()
{
  GPIO.out_w1tc = LEFT_STEP_MASK;
}

inline void IRAM_ATTR rightStepHigh()
{
  GPIO.out_w1ts = RIGHT_STEP_MASK;
}

inline void IRAM_ATTR rightStepLow()
{
  GPIO.out_w1tc = RIGHT_STEP_MASK;
}


// ================================================================
// STEP ISR
// ================================================================

void ARDUINO_ISR_ATTR onStepTimer()
{
  if (leftPulseHigh)
  {
    leftStepLow();
    leftPulseHigh = false;
  }

  if (rightPulseHigh)
  {
    rightStepLow();
    rightPulseHigh = false;
  }


  uint32_t leftInc =
    leftPhaseIncrement;

  if (leftInc != 0)
  {
    uint32_t oldPhase =
      leftPhase;

    uint32_t newPhase =
      oldPhase + leftInc;

    leftPhase =
      newPhase;

    if (newPhase < oldPhase)
    {
      leftStepHigh();

      leftPulseHigh = true;

      robotStepPosition +=
        leftPositionStepSign;
    }
  }


  uint32_t rightInc =
    rightPhaseIncrement;

  if (rightInc != 0)
  {
    uint32_t oldPhase =
      rightPhase;

    uint32_t newPhase =
      oldPhase + rightInc;

    rightPhase =
      newPhase;

    if (newPhase < oldPhase)
    {
      rightStepHigh();

      rightPulseHigh = true;
    }
  }
}


// ================================================================
// RPM -> PHASE INCREMENT
// ================================================================

uint32_t rpmToPhaseIncrement(float rpm)
{
  float stepsPerSecond =
    fabs(rpm) *
    STEPS_PER_REV /
    60.0f;

  if (stepsPerSecond < 0.01f)
    return 0;

  const double PHASE_SCALE =
    4294967296.0;

  double value =
    ((double)stepsPerSecond /
     (double)STEP_ISR_HZ)
    *
    PHASE_SCALE;

  if (value < 1.0)
    value = 1.0;

  if (value > 4294967295.0)
    value = 4294967295.0;

  return (uint32_t)value;
}


// ================================================================
// SET ONE MOTOR
// ================================================================

void setOneMotor(
  bool isLeft,
  StepperState &motor,
  float rpm)
{
  if (fabs(rpm) < 0.01f)
  {
    portENTER_CRITICAL(&stepMux);

    if (isLeft)
    {
      leftPhaseIncrement = 0;
      leftPositionStepSign = 0;
    }
    else
    {
      rightPhaseIncrement = 0;
    }

    portEXIT_CRITICAL(&stepMux);

    return;
  }


  int logicalDirection =
    (rpm > 0.0f)
      ? +1
      : -1;


  if (isLeft)
  {
    portENTER_CRITICAL(&stepMux);

    leftPositionStepSign =
      -logicalDirection;

    portEXIT_CRITICAL(&stepMux);
  }


  if (logicalDirection !=
      motor.lastLogicalDirection)
  {
    portENTER_CRITICAL(&stepMux);

    if (isLeft)
      leftPhaseIncrement = 0;
    else
      rightPhaseIncrement = 0;

    portEXIT_CRITICAL(&stepMux);


    int physicalDirection =
      logicalDirection *
      motor.motorSign;


    digitalWrite(
      motor.dirPin,
      physicalDirection > 0
        ? HIGH
        : LOW
    );


    delayMicroseconds(3);


    motor.lastLogicalDirection =
      logicalDirection;
  }


  uint32_t increment =
    rpmToPhaseIncrement(rpm);


  portENTER_CRITICAL(&stepMux);

  if (isLeft)
    leftPhaseIncrement = increment;
  else
    rightPhaseIncrement = increment;

  portEXIT_CRITICAL(&stepMux);
}


// ================================================================
// SET BOTH MOTORS
// ================================================================

void setMotorRPM(
  float leftRPM,
  float rightRPM)
{
  leftRPM =
    constrain(
      leftRPM,
      -MAX_DESIRED_RPM,
      MAX_DESIRED_RPM
    );

  rightRPM =
    constrain(
      rightRPM,
      -MAX_DESIRED_RPM,
      MAX_DESIRED_RPM
    );


  setOneMotor(
    true,
    leftMotor,
    leftRPM
  );


  setOneMotor(
    false,
    rightMotor,
    rightRPM
  );
}


// ================================================================
// MOTOR SLEW LIMIT
// ================================================================

void updateActualMotorRPM()
{
  float difference =
    desiredMotorRPM -
    actualMotorRPM;


  if (difference >
      MOTOR_RPM_SLEW_PER_LOOP)
  {
    actualMotorRPM +=
      MOTOR_RPM_SLEW_PER_LOOP;
  }
  else if (difference <
           -MOTOR_RPM_SLEW_PER_LOOP)
  {
    actualMotorRPM -=
      MOTOR_RPM_SLEW_PER_LOOP;
  }
  else
  {
    actualMotorRPM =
      desiredMotorRPM;
  }


  actualMotorRPM =
    constrain(
      actualMotorRPM,
      -MAX_DESIRED_RPM,
      MAX_DESIRED_RPM
    );
}


// ================================================================
// POSITION HELPERS
// ================================================================

void zeroPosition()
{
  portENTER_CRITICAL(&stepMux);

  robotStepPosition = 0;

  portEXIT_CRITICAL(&stepMux);

  returnDirection = 0;
  returnIntegral = 0.0f;
}


int32_t getRobotPosition()
{
  int32_t p;

  portENTER_CRITICAL(&stepMux);

  p = robotStepPosition;

  portEXIT_CRITICAL(&stepMux);

  return p;
}


// ================================================================
// STOP
// ================================================================

void stopMotors()
{
  desiredMotorRPM = 0.0f;
  actualMotorRPM  = 0.0f;
  remoteSpeedRPM = 0.0f;
  remoteTurnRPM = 0.0f;
  remoteWasMoving = false;

  filteredMotorRPM = 0.0f;

  returnDirection = 0;
  returnIntegral = 0.0f;


  portENTER_CRITICAL(&stepMux);

  leftPhaseIncrement  = 0;
  rightPhaseIncrement = 0;

  leftPhase  = 0;
  rightPhase = 0;

  leftPositionStepSign = 0;

  portEXIT_CRITICAL(&stepMux);


  leftMotor.lastLogicalDirection  = 0;
  rightMotor.lastLogicalDirection = 0;


  leftStepLow();
  rightStepLow();
}


// ================================================================
// BMI HELPERS
// ================================================================

void writeBMI(
  uint8_t reg,
  uint8_t value)
{
  Wire.beginTransmission(BMI_ADDR);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
}


uint8_t readBMI8(uint8_t reg)
{
  Wire.beginTransmission(BMI_ADDR);

  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  Wire.requestFrom(
    BMI_ADDR,
    (uint8_t)1
  );

  if (!Wire.available())
    return 0xFF;

  return Wire.read();
}


// ================================================================
// BMI160 INIT
// ================================================================

bool initBMI160()
{
  uint8_t id =
    readBMI8(0x00);


  Serial.print("BMI160 ID = 0x");
  Serial.println(id, HEX);


  if (id != 0xD1)
    return false;


  writeBMI(0x7E, 0x11);
  delay(100);

  writeBMI(0x7E, 0x15);
  delay(100);


  writeBMI(0x40, 0x2B);
  writeBMI(0x41, 0x03);

  writeBMI(0x42, 0x2B);
  writeBMI(0x43, 0x02);


  delay(100);


  return true;
}


// ================================================================
// READ BMI160
// ================================================================

bool readBMI(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz)
{
  Wire.beginTransmission(BMI_ADDR);

  Wire.write(0x0C);

  if (Wire.endTransmission(false) != 0)
    return false;

  Wire.requestFrom(
    BMI_ADDR,
    (uint8_t)12
  );

  if (Wire.available() < 12)
    return false;


  int16_t rgx =
    (int16_t)
    (Wire.read() |
    (Wire.read() << 8));

  int16_t rgy =
    (int16_t)
    (Wire.read() |
    (Wire.read() << 8));

  int16_t rgz =
    (int16_t)
    (Wire.read() |
    (Wire.read() << 8));


  int16_t rax =
    (int16_t)
    (Wire.read() |
    (Wire.read() << 8));

  int16_t ray =
    (int16_t)
    (Wire.read() |
    (Wire.read() << 8));

  int16_t raz =
    (int16_t)
    (Wire.read() |
    (Wire.read() << 8));


  ax = rax / 16384.0f;
  ay = ray / 16384.0f;
  az = raz / 16384.0f;

  gx = rgx / GYRO_SCALE;
  gy = rgy / GYRO_SCALE;
  gz = rgz / GYRO_SCALE;


  return true;
}


// ================================================================
// ACCEL ANGLE
// ================================================================

float getAccelAngle(
  float ay,
  float az)
{
  return
    ACCEL_SIGN *
    atan2(az, ay) *
    180.0f / PI;
}


// ================================================================
// GYRO CALIBRATION
// ================================================================

void calibrateGyro()
{
  Serial.println();
  Serial.println("Keep robot still.");
  Serial.println("Calibrating gyro...");


  float total = 0.0f;
  int count = 0;


  for (int i = 0;
       i < 500;
       i++)
  {
    float ax, ay, az;
    float gx, gy, gz;


    if (readBMI(
          ax, ay, az,
          gx, gy, gz))
    {
      total += gx;
      count++;
    }


    delay(2);
  }


  if (count > 0)
  {
    gyroZero =
      total / count;
  }


  Serial.print("Gyro zero = ");
  Serial.println(gyroZero, 4);
}


// ================================================================
// ARM
// ================================================================

void armRobot()
{
  float ax, ay, az;
  float gx, gy, gz;


  if (!readBMI(
        ax, ay, az,
        gx, gy, gz))
  {
    Serial.println("BMI read failed");
    return;
  }


  accelZero =
    getAccelAngle(
      ay,
      az
    );


  accelAngle = 0.0f;
  angleDeg   = 0.0f;


  potZero =
    analogRead(
      TRIM_POT_PIN
    );


  trimAngle = 0.0f;


  stopMotors();

  zeroPosition();


  armed = true;


  Serial.println();
  Serial.println(
    "*** ARMED - POSITION ZEROED ***"
  );
}


// ================================================================
// DISARM
// ================================================================

void disarmRobot()
{
  if (!armed)
    return;


  armed = false;

  stopMotors();


  Serial.println();
  Serial.println("*** DISARMED ***");
}


// ================================================================
// MAIN CONTROL
// ================================================================

void controlUpdate()
{
  float ax, ay, az;
  float gx, gy, gz;


  if (!readBMI(
        ax, ay, az,
        gx, gy, gz))
  {
    return;
  }


  // ============================================================
  // IMU
  // ============================================================

  gyroRate =
    GYRO_SIGN *
    (gx - gyroZero);


  float absoluteAccelAngle =
    getAccelAngle(
      ay,
      az
    );


  accelAngle =
    absoluteAccelAngle -
    accelZero;


  angleDeg =

      FILTER_GYRO *
      (
        angleDeg +
        gyroRate * DT
      )

    +

      (1.0f - FILTER_GYRO) *
      accelAngle;


  // ============================================================
  // FALL SAFETY
  // ============================================================

  if (fabs(angleDeg) >
      FALL_ANGLE)
  {
    disarmRobot();

    return;
  }


  // ============================================================
  // TRIM POT
  // ============================================================

  int potRaw =
    analogRead(
      TRIM_POT_PIN
    );


  trimAngle =
    ((potRaw - potZero) /
     2047.0f)
    *
    TRIM_RANGE_DEG;


  trimAngle =
    constrain(
      trimAngle,
      -TRIM_RANGE_DEG,
      TRIM_RANGE_DEG
    );


  // ============================================================
  // ROBOT SPEED
  // ============================================================

  float actualSpeed =
    -actualMotorRPM;


  filteredMotorRPM =

      filteredMotorRPM *
      SPEED_FILTER_OLD

    +

      actualSpeed *
      SPEED_FILTER_NEW;

  // Copy the latest radio command atomically; never balance in WiFi callback.
  ControlPacket cmd;
  uint32_t packetAt;
  bool havePacket;
  portENTER_CRITICAL(&remoteMux);
  cmd = latestRemote;
  packetAt = latestRemoteAt;
  havePacket = receivedRemote;
  portEXIT_CRITICAL(&remoteMux);

  bool remoteActive = havePacket &&
    (uint32_t)(millis() - packetAt) <= REMOTE_TIMEOUT_MS && cmd.armed;
  float requestedSpeed = remoteActive ?
    cmd.throttle * (REMOTE_MAX_SPEED_RPM / 1000.0f) : 0.0f;
  float requestedTurn = remoteActive ?
    cmd.steering * (REMOTE_MAX_TURN_RPM / 1000.0f) : 0.0f;
  remoteSpeedRPM = approachValue(remoteSpeedRPM, requestedSpeed,
                                 REMOTE_SPEED_RAMP_RPM_S * DT);
  remoteTurnRPM = approachValue(remoteTurnRPM, requestedTurn,
                                REMOTE_TURN_RAMP_RPM_S * DT);
  bool remoteMoving = fabsf(remoteSpeedRPM) > 0.05f ||
                      fabsf(remoteTurnRPM) > 0.05f;

  // Start a fresh home position after joystick motion ends. The original
  // automatic return remains active when the joystick is neutral.
  if (remoteWasMoving && !remoteMoving) zeroPosition();
  remoteWasMoving = remoteMoving;


  // ============================================================
  // POSITION RETURN STATE
  // ============================================================

  int32_t position =
    getRobotPosition();


  // Start return beyond outer boundary.

  if (remoteMoving)
  {
    returnDirection = 0;
    returnIntegral = 0.0f;
  }
  else if (returnDirection == 0)
  {
    if (position >
        RETURN_START_STEPS)
    {
      returnDirection = -1;
      returnIntegral = 0.0f;
    }
    else if (position <
             -RETURN_START_STEPS)
    {
      returnDirection = +1;
      returnIntegral = 0.0f;
    }
  }


  // Stop return near home.

  if (returnDirection != 0)
  {
    if (abs(position) <
        RETURN_STOP_STEPS)
    {
      returnDirection = 0;
      returnIntegral = 0.0f;
    }
  }


  // ============================================================
  // TARGET RETURN SPEED
  // ============================================================

  float targetSpeedRPM =
    remoteMoving ? remoteSpeedRPM : returnDirection * RETURN_RPM;


  // ============================================================
  // SPEED ERROR
  // ============================================================

  float speedError =
    targetSpeedRPM -
    filteredMotorRPM;


  // ============================================================
  // RETURN INTEGRAL
  // ============================================================

  float returnIntegralAngle = 0.0f;


  if (returnDirection != 0)
  {
    // Normal accumulation everywhere during return.
    returnIntegral +=
      speedError *
      DT;


    // Once we get close to home, rapidly bleed off
    // the stored correction so it does not carry
    // the robot through home.
    if (abs(position) <
        RETURN_SLOW_STEPS)
    {
      returnIntegral *=
        RETURN_I_NEAR_HOME_DECAY;
    }


    // Anti-windup limit corresponding to +/-1 degree.
    if (RETURN_KI > 0.0f)
    {
      float maxIntegral =
        MAX_RETURN_I_ANGLE /
        RETURN_KI;


      returnIntegral =
        constrain(
          returnIntegral,
          -maxIntegral,
          maxIntegral
        );
    }


    returnIntegralAngle =
      RETURN_KI *
      returnIntegral;


    returnIntegralAngle =
      constrain(
        returnIntegralAngle,
        -MAX_RETURN_I_ANGLE,
        MAX_RETURN_I_ANGLE
      );
  }
  else
  {
    returnIntegral = 0.0f;
  }


  // ============================================================
  // SPEED LOOP -> TARGET ANGLE
  // ============================================================

  float speedTargetAngle =

      (remoteMoving ? REMOTE_SPEED_KP : SPEED_KP) *
      speedError

    +

      returnIntegralAngle;


  speedTargetAngle =
    constrain(
      speedTargetAngle,
      remoteMoving ? -REMOTE_MAX_TILT_DEG : -MAX_TARGET_ANGLE,
      remoteMoving ? REMOTE_MAX_TILT_DEG : MAX_TARGET_ANGLE
    );


  // ============================================================
  // FINAL TARGET ANGLE
  // ============================================================

  float targetAngle =
    trimAngle +
    speedTargetAngle;


  // ============================================================
  // ANGLE PD -> MOTOR ACCELERATION
  // ============================================================

  float angleError =
    targetAngle -
    angleDeg;


  float motorAcceleration =

      ANGLE_KP *
      angleError

    -

      ANGLE_KD *
      gyroRate;


  // ============================================================
  // ACCELERATION -> DESIRED RPM
  // ============================================================

  desiredMotorRPM +=
    motorAcceleration *
    DT;


  desiredMotorRPM =
    constrain(
      desiredMotorRPM,
      -MAX_DESIRED_RPM,
      MAX_DESIRED_RPM
    );


  // ============================================================
  // FINAL MOTOR SLEW
  // ============================================================

  updateActualMotorRPM();


  // ============================================================
  // MINIMUM ACTIVE RPM
  // ============================================================

  float outputRPM =
    actualMotorRPM;


  if (outputRPM > 0.0f &&
      outputRPM < MIN_ACTIVE_RPM)
  {
    outputRPM =
      MIN_ACTIVE_RPM;
  }
  else if (outputRPM < 0.0f &&
           outputRPM > -MIN_ACTIVE_RPM)
  {
    outputRPM =
      -MIN_ACTIVE_RPM;
  }


  // ============================================================
  // OUTPUT
  // ============================================================

  setMotorRPM(
    outputRPM - remoteTurnRPM,
    outputRPM + remoteTurnRPM
  );
}


// ================================================================
// BUTTON
// ================================================================

void checkButton()
{
  static bool oldState = HIGH;
  static uint32_t lastChange = 0;


  bool state =
    digitalRead(
      ARM_BUTTON_PIN
    );


  if (state != oldState &&
      millis() - lastChange > 40)
  {
    lastChange =
      millis();

    oldState =
      state;


    if (state == LOW)
    {
      if (armed)
      {
        disarmRobot();
      }
      else
      {
        armRobot();
      }
    }
  }
}


// ================================================================
// SERIAL
// ================================================================

void checkSerial()
{
  if (!Serial.available())
    return;


  char c =
    Serial.read();


  if (c == 'A' ||
      c == 'a')
  {
    armRobot();
  }


  if (c == 'X' ||
      c == 'x')
  {
    disarmRobot();
  }


  if (c == 'P' ||
      c == 'p')
  {
    int32_t p =
      getRobotPosition();


    Serial.print("Position=");
    Serial.print(p);

    Serial.print(" steps  rev=");
    Serial.print(
      (float)p /
      STEPS_PER_REV,
      2
    );

    Serial.print("  returnDir=");
    Serial.print(returnDirection);

    Serial.print("  speed=");
    Serial.print(
      -actualMotorRPM,
      2
    );

    Serial.print("  filtered=");
    Serial.print(
      filteredMotorRPM,
      2
    );

    Serial.print("  target=");
    Serial.print(
      returnDirection *
      RETURN_RPM,
      2
    );

    Serial.print("  I=");
    Serial.print(
      returnIntegral,
      2
    );

    Serial.print("  Iangle=");
    Serial.println(
      constrain(
        RETURN_KI *
        returnIntegral,
        -MAX_RETURN_I_ANGLE,
        MAX_RETURN_I_ANGLE
      ),
      3
    );
  }
}


// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(115200);

  delay(500);

  // HW675 transmits broadcast ESP-NOW packets on channel 1.
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);
  if (esp_now_init() != ESP_OK)
    Serial.println("WARNING: ESP-NOW initialization failed");
  else if (esp_now_register_recv_cb(onRemotePacket) != ESP_OK)
    Serial.println("WARNING: ESP-NOW receive callback failed");
  else
    Serial.println("ESP-NOW joystick receiver ready (channel 1)");


  pinMode(
    LEFT_STEP_PIN,
    OUTPUT
  );

  pinMode(
    RIGHT_STEP_PIN,
    OUTPUT
  );

  pinMode(
    LEFT_DIR_PIN,
    OUTPUT
  );

  pinMode(
    RIGHT_DIR_PIN,
    OUTPUT
  );

  pinMode(
    ENABLE_PIN,
    OUTPUT
  );

  pinMode(
    ARM_BUTTON_PIN,
    INPUT_PULLUP
  );

  pinMode(
    TRIM_POT_PIN,
    INPUT
  );


  digitalWrite(
    LEFT_STEP_PIN,
    LOW
  );

  digitalWrite(
    RIGHT_STEP_PIN,
    LOW
  );

  digitalWrite(
    ENABLE_PIN,
    LOW
  );


  // ============================================================
  // 40 kHz STEP TIMER
  // ============================================================

  stepTimer =
    timerBegin(
      STEP_TIMER_BASE_HZ
    );


  if (stepTimer == NULL)
  {
    Serial.println(
      "Timer initialization failed"
    );

    while (true)
      delay(1000);
  }


  timerAttachInterrupt(
    stepTimer,
    &onStepTimer
  );


  timerAlarm(
    stepTimer,
    STEP_TIMER_TICKS,
    true,
    0
  );


  // ============================================================
  // BMI160
  // ============================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


  Wire.setClock(
    400000
  );


  Serial.println();
  Serial.println(
    "ESP32 Balance Bot - Near Home Integral Bleed"
  );


  Serial.print("SPEED_KP = ");
  Serial.println(
    SPEED_KP,
    3
  );


  Serial.print("RETURN_RPM = ");
  Serial.println(
    RETURN_RPM,
    2
  );


  Serial.print("RETURN_KI = ");
  Serial.println(
    RETURN_KI,
    3
  );


  Serial.print("MAX_RETURN_I_ANGLE = ");
  Serial.println(
    MAX_RETURN_I_ANGLE,
    2
  );


  Serial.print("Return starts +/- ");
  Serial.print(
    (float)RETURN_START_STEPS /
    STEPS_PER_REV,
    2
  );

  Serial.println(" rev");


  Serial.print("Integral bleed begins +/- ");
  Serial.print(
    (float)RETURN_SLOW_STEPS /
    STEPS_PER_REV,
    2
  );

  Serial.println(" rev");


  Serial.print("Return stops +/- ");
  Serial.print(
    (float)RETURN_STOP_STEPS /
    STEPS_PER_REV,
    2
  );

  Serial.println(" rev");


  if (!initBMI160())
  {
    Serial.println(
      "BMI160 NOT FOUND"
    );

    while (true)
      delay(1000);
  }


  calibrateGyro();


  Serial.println();
  Serial.println(
    "Short press = ARM / STOP"
  );

  Serial.println(
    "P = print position/status"
  );

  Serial.println("Remote button enables joystick commands; bot button arms balancing");
  Serial.println("Radio loss: joystick returns to neutral in 300 ms + ramp");
}


// ================================================================
// LOOP
// ================================================================

void loop()
{
  checkButton();

  checkSerial();


  static uint32_t nextControl = 0;


  uint32_t now =
    micros();


  if ((int32_t)
      (now - nextControl) >= 0)
  {
    nextControl +=
      CONTROL_PERIOD_US;


    if ((int32_t)
        (now - nextControl) >
        (int32_t)CONTROL_PERIOD_US)
    {
      nextControl =
        now +
        CONTROL_PERIOD_US;
    }


    if (armed)
    {
      controlUpdate();
    }
  }
}

