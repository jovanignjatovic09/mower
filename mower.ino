// Raspberry Pi Pico + 2x BTS7960 + ELRS PWM receiver
// Tank steer mixer with HC-05 Bluetooth debug and faster asymmetric PWM ramping
//
// CH1 = forward/back
// CH2 = left/right
// CH3 = mode switch:
//   ~1000 = disarmed
//   ~1500 = limited speed mode
//   ~2000 = cruise/full speed mode
//
// Receiver failsafe should output 0 on signal loss.
//
// EN behavior:
// BTS7960 R_EN and L_EN are driven HIGH all the time.
// If you physically tied EN pins HIGH, that is also fine.
// Failsafe/disarm stops by sending 0 PWM to all BTS7960 PWM pins.
//
// HC-05 debug:
// Pico GP0 / physical pin 1 -> HC-05 RXD
// Common GND required.

#include <Arduino.h>

// ===================== USER CONFIG =====================

// Receiver input pins
const int CH1_PIN = 2;   // Forward/back
const int CH2_PIN = 3;   // Left/right
const int CH3_PIN = 4;   // Mode switch

// Left BTS7960
const int LEFT_RPWM_PIN = 10;
const int LEFT_LPWM_PIN = 11;
const int LEFT_REN_PIN  = 14;
const int LEFT_LEN_PIN  = 15;

// Right BTS7960
const int RIGHT_RPWM_PIN = 12;
const int RIGHT_LPWM_PIN = 13;
const int RIGHT_REN_PIN  = 16;
const int RIGHT_LEN_PIN  = 17;

// HC-05 Bluetooth debug
const bool USE_BT_DEBUG = true;
const int BT_TX_PIN = 0;           // Pico GP0 -> HC-05 RXD
const unsigned long BT_DEBUG_MS = 200;

// Pulse settings
const int PWM_MIN_VALID = 900;
const int PWM_MAX_VALID = 2100;
const int PWM_CENTER    = 1500;

// Stick deadzone around center
const int STICK_DEADZONE_US = 45;

// Mode switch bands
const int MODE_DISARM_MAX  = 1250;
const int MODE_LIMITED_MAX = 1750;

// Output limits
const int LIMITED_SPEED_PERCENT = 40;

// Start with 60 while testing cutouts.
// Raise to 80, 90, or 100 after it behaves.
const int CRUISE_SPEED_PERCENT = 60;

// Steering tuning
// 100 = normal steering
// 70 = softer steering
const int STEERING_PERCENT = 100;

// Minimum output needed to overcome motor deadband.
// Keep 0 while debugging cutouts.
const int MIN_MOTOR_PWM = 0;

// PWM frequency.
// 1000 Hz is usually easier on cheap BTS7960 modules than 20 kHz.
const int MOTOR_PWM_FREQ_HZ = 1000;

// Signal timeout
const unsigned long SIGNAL_TIMEOUT_MS = 200;

// Debug print interval over USB
const unsigned long DEBUG_PRINT_MS = 100;

// Faster asymmetric ramping.
// Acceleration is quick, stopping is much faster.
const int PWM_RAMP_UP_STEP = 30;
const int PWM_RAMP_DOWN_STEP = 120;

// Motor direction correction — kept from your pasted code
const bool INVERT_LEFT_MOTOR  = true;
const bool INVERT_RIGHT_MOTOR = true;

// ===================== INTERNAL STATE =====================

unsigned long lastGoodFrameTime = 0;
unsigned long lastDebugPrint = 0;
unsigned long lastBtDebugPrint = 0;

int ch1 = 0;
int ch2 = 0;
int ch3 = 0;

int leftPWMActual = 0;
int rightPWMActual = 0;

enum DriveMode {
  MODE_SIGNAL_LOST,
  MODE_DISARMED,
  MODE_LIMITED,
  MODE_CRUISE
};

// ===================== HELPERS =====================

void keepDriversEnabled() {
  digitalWrite(LEFT_REN_PIN, HIGH);
  digitalWrite(LEFT_LEN_PIN, HIGH);
  digitalWrite(RIGHT_REN_PIN, HIGH);
  digitalWrite(RIGHT_LEN_PIN, HIGH);
}

void stopMotorsImmediate() {
  leftPWMActual = 0;
  rightPWMActual = 0;

  analogWrite(LEFT_RPWM_PIN, 0);
  analogWrite(LEFT_LPWM_PIN, 0);
  analogWrite(RIGHT_RPWM_PIN, 0);
  analogWrite(RIGHT_LPWM_PIN, 0);
}

int readPulseUS(int pin) {
  unsigned long pulse = pulseIn(pin, HIGH, 25000);

  if (pulse == 0) {
    return 0;
  }

  return (int)pulse;
}

bool pulseValid(int us) {
  return us >= PWM_MIN_VALID && us <= PWM_MAX_VALID;
}

bool frameValid() {
  return pulseValid(ch1) && pulseValid(ch2) && pulseValid(ch3);
}

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int applyDeadzoneAndMap(int pulse) {
  if (!pulseValid(pulse)) {
    return 0;
  }

  int centered = pulse - PWM_CENTER;

  if (abs(centered) <= STICK_DEADZONE_US) {
    return 0;
  }

  int output = 0;

  if (centered > 0) {
    output = map(centered, STICK_DEADZONE_US, 500, 0, 1000);
  } else {
    output = map(centered, -STICK_DEADZONE_US, -500, 0, -1000);
  }

  return clampInt(output, -1000, 1000);
}

DriveMode getMode(bool timedOut) {
  if (!frameValid() || timedOut) {
    return MODE_SIGNAL_LOST;
  }

  if (ch3 < MODE_DISARM_MAX) {
    return MODE_DISARMED;
  }

  if (ch3 < MODE_LIMITED_MAX) {
    return MODE_LIMITED;
  }

  return MODE_CRUISE;
}

const char* modeName(DriveMode mode) {
  switch (mode) {
    case MODE_SIGNAL_LOST: return "SIGNAL_LOST";
    case MODE_DISARMED:    return "DISARMED";
    case MODE_LIMITED:     return "LIMITED";
    case MODE_CRUISE:      return "CRUISE";
    default:               return "UNKNOWN";
  }
}

int scaleByPercent(int value, int percent) {
  return value * percent / 100;
}

int convert1000To255(int value) {
  value = clampInt(value, -1000, 1000);

  int pwm = map(abs(value), 0, 1000, 0, 255);

  if (value < 0) {
    pwm = -pwm;
  }

  return pwm;
}

int applyMinimumMotorPWM(int value) {
  if (value == 0) {
    return 0;
  }

  int sign = value > 0 ? 1 : -1;
  int magnitude = abs(value);

  if (magnitude < MIN_MOTOR_PWM) {
    magnitude = MIN_MOTOR_PWM;
  }

  magnitude = clampInt(magnitude, 0, 255);

  return sign * magnitude;
}

int rampTowardAsymmetric(int current, int target, int upStep, int downStep) {
  int step;

  // If target magnitude is smaller than current magnitude, decelerate faster.
  if (abs(target) < abs(current)) {
    step = downStep;
  } else {
    step = upStep;
  }

  // If changing direction, first decelerate fast toward zero.
  if ((current > 0 && target < 0) || (current < 0 && target > 0)) {
    target = 0;
    step = downStep;
  }

  if (current < target) {
    current += step;
    if (current > target) {
      current = target;
    }
  } else if (current > target) {
    current -= step;
    if (current < target) {
      current = target;
    }
  }

  return current;
}

void driveOneBTS7960(int rpwmPin, int lpwmPin, int speed255, bool invert) {
  speed255 = clampInt(speed255, -255, 255);

  if (invert) {
    speed255 = -speed255;
  }

  if (speed255 > 0) {
    analogWrite(rpwmPin, speed255);
    analogWrite(lpwmPin, 0);
  } else if (speed255 < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed255);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void printDebugTo(Stream &out,
                  DriveMode mode,
                  bool timedOut,
                  int throttle,
                  int steering,
                  int leftMixed,
                  int rightMixed,
                  int leftPWMTarget,
                  int rightPWMTarget) {
  out.print("CH1=");
  out.print(ch1);

  out.print(" CH2=");
  out.print(ch2);

  out.print(" CH3=");
  out.print(ch3);

  out.print(" MODE=");
  out.print(modeName(mode));

  out.print(" THR=");
  out.print(throttle);

  out.print(" STR=");
  out.print(steering);

  out.print(" Lmix=");
  out.print(leftMixed);

  out.print(" Rmix=");
  out.print(rightMixed);

  out.print(" Ltgt=");
  out.print(leftPWMTarget);

  out.print(" Rtgt=");
  out.print(rightPWMTarget);

  out.print(" Lact=");
  out.print(leftPWMActual);

  out.print(" Ract=");
  out.print(rightPWMActual);

  out.print(" EN=HIGH");

  if (timedOut) {
    out.print(" TIMEOUT");
  }

  out.println();
}

// ===================== SETUP =====================

void setup() {
  // PWM pins low ASAP
  pinMode(LEFT_RPWM_PIN, OUTPUT);
  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_LPWM_PIN, OUTPUT);

  analogWrite(LEFT_RPWM_PIN, 0);
  analogWrite(LEFT_LPWM_PIN, 0);
  analogWrite(RIGHT_RPWM_PIN, 0);
  analogWrite(RIGHT_LPWM_PIN, 0);

  // EN pins HIGH all the time after boot
  pinMode(LEFT_REN_PIN, OUTPUT);
  pinMode(LEFT_LEN_PIN, OUTPUT);
  pinMode(RIGHT_REN_PIN, OUTPUT);
  pinMode(RIGHT_LEN_PIN, OUTPUT);

  keepDriversEnabled();

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);

  analogWriteFreq(MOTOR_PWM_FREQ_HZ);
  analogWriteRange(255);

  stopMotorsImmediate();
  keepDriversEnabled();

  lastGoodFrameTime = millis();

  Serial.begin(115200);
  delay(1000);

  if (USE_BT_DEBUG) {
    Serial1.setTX(BT_TX_PIN);   // GP0 -> HC-05 RXD
    Serial1.begin(9600);
    delay(100);
    Serial1.println("BT debug online");
  }

  Serial.println("Pico ELRS BTS7960 tank mixer started");
  Serial.println("EN pins always HIGH. Failsafe = PWM 0.");
  Serial.println("PWM freq = 1000 Hz. Fast asymmetric ramping enabled.");
}

// ===================== LOOP =====================

void loop() {
  keepDriversEnabled();

  ch1 = readPulseUS(CH1_PIN);
  ch2 = readPulseUS(CH2_PIN);
  ch3 = readPulseUS(CH3_PIN);

  bool validNow = frameValid();

  if (validNow) {
    lastGoodFrameTime = millis();
  }

  bool timedOut = millis() - lastGoodFrameTime > SIGNAL_TIMEOUT_MS;

  DriveMode mode = getMode(timedOut);

  int throttle = 0;
  int steering = 0;

  int leftMixed = 0;
  int rightMixed = 0;

  int leftPWMTarget = 0;
  int rightPWMTarget = 0;

  if (mode == MODE_SIGNAL_LOST || mode == MODE_DISARMED) {
    // Failsafe/disarm: instant no PWM output
    leftPWMTarget = 0;
    rightPWMTarget = 0;

    leftPWMActual = 0;
    rightPWMActual = 0;

    driveOneBTS7960(LEFT_RPWM_PIN, LEFT_LPWM_PIN, 0, INVERT_LEFT_MOTOR);
    driveOneBTS7960(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, 0, INVERT_RIGHT_MOTOR);
  } else {
    throttle = applyDeadzoneAndMap(ch1);
    steering = applyDeadzoneAndMap(ch2);

    steering = scaleByPercent(steering, STEERING_PERCENT);

    // Tank mix
    leftMixed  = throttle + steering;
    rightMixed = throttle - steering;

    leftMixed = clampInt(leftMixed, -1000, 1000);
    rightMixed = clampInt(rightMixed, -1000, 1000);

    if (mode == MODE_LIMITED) {
      leftMixed = scaleByPercent(leftMixed, LIMITED_SPEED_PERCENT);
      rightMixed = scaleByPercent(rightMixed, LIMITED_SPEED_PERCENT);
    } else if (mode == MODE_CRUISE) {
      leftMixed = scaleByPercent(leftMixed, CRUISE_SPEED_PERCENT);
      rightMixed = scaleByPercent(rightMixed, CRUISE_SPEED_PERCENT);
    }

    leftPWMTarget = convert1000To255(leftMixed);
    rightPWMTarget = convert1000To255(rightMixed);

    leftPWMTarget = applyMinimumMotorPWM(leftPWMTarget);
    rightPWMTarget = applyMinimumMotorPWM(rightPWMTarget);

    leftPWMActual = rampTowardAsymmetric(leftPWMActual, leftPWMTarget, PWM_RAMP_UP_STEP, PWM_RAMP_DOWN_STEP);
    rightPWMActual = rampTowardAsymmetric(rightPWMActual, rightPWMTarget, PWM_RAMP_UP_STEP, PWM_RAMP_DOWN_STEP);

    driveOneBTS7960(LEFT_RPWM_PIN, LEFT_LPWM_PIN, leftPWMActual, INVERT_LEFT_MOTOR);
    driveOneBTS7960(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, rightPWMActual, INVERT_RIGHT_MOTOR);
  }

  if (millis() - lastDebugPrint >= DEBUG_PRINT_MS) {
    lastDebugPrint = millis();
    printDebugTo(Serial, mode, timedOut, throttle, steering, leftMixed, rightMixed, leftPWMTarget, rightPWMTarget);
  }

  if (USE_BT_DEBUG && millis() - lastBtDebugPrint >= BT_DEBUG_MS) {
    lastBtDebugPrint = millis();
    printDebugTo(Serial1, mode, timedOut, throttle, steering, leftMixed, rightMixed, leftPWMTarget, rightPWMTarget);
  }
}
