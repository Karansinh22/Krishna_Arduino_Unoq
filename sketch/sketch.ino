/**
 * ============================================================================
 * Project: Autonomous Agrobot — Arduino UNO Q Firmware
 * Challenge: Arduino Physical AI Challenge India 2026 (Team Krishna)
 * Track: Industrial AI Sustainability
 * 
 * Hardware Architecture:
 * - Central Board: Arduino UNO Q (ABX00087) — Zephyr MCU Firmware Bridge
 * - Motor Driver: Dual H-Bridge DC Motor Driver (Pins D2-D5)
 * - Robotic Arm: 4-Axis Servo Positioning (Pins D6, D9, D10, D11)
 * - Herbicide Actuators: 2x Diaphragm Water Pumps via Relays (Pins D7, D8)
 * - Visual Guidance: 2x Target Alignment Lasers (Pins GPIO 12, GPIO 13)
 * ============================================================================
 */

#include <Arduino_RouterBridge.h>

// ============================================================================
// PIN DEFINITIONS & HARDWARE CONSTANTS
// ============================================================================

// Motor Driver H-Bridge Pins
const int L_DIR = 2; // Left Motor Direction (HIGH = Forward, LOW = Reverse)
const int L_PWM = 3; // Left Motor Speed (PWM 0-255)
const int R_DIR = 4; // Right Motor Direction (HIGH = Forward, LOW = Reverse)
const int R_PWM = 5; // Right Motor Speed (PWM 0-255)

// Global Speed & Wheel Trim Compensation
int motorSpeed = 10;  // Default startup speed percentage (10%)
int leftTrim   = 5;   // Left wheel PWM offset (compensation for motor drift)
int rightTrim  = 0;   // Right wheel PWM offset

// 4-Axis Robotic Arm Servo Pins (Software Servo PWM)
const int SERVO_PINS[4]  = {6, 9, 10, 11}; // Servos 1-4: Base Pan, Nozzle Pitch, Elevation, Deflector
int       servoAngles[4] = {90, 90, 90, 90}; // Default startup angles (90° Home)

// Herbicide Spray Relay Pins (Active HIGH)
const int RELAY_PINS[2] = {7, 8}; // Relay 1: Primary Diaphragm Pump, Relay 2: Aux Valve

// Targeting Laser Pins (Active HIGH)
const int LASER_PINS[2] = {12, 13}; // Laser 1: Target Beam, Laser 2: Alignment Marker

// Servo Timing Constants
#define NUM_PULSES 35         // Pulses sent per servo write command
#define SERVO_PERIOD_MS 20    // Standard 50Hz servo period (20ms)

// ============================================================================
// MOTOR PRIMITIVES & TRIM FUNCTIONS
// ============================================================================

/**
 * Sets default motor speed percentage (0 - 100%).
 */
void setMotorSpeed(int speed) {
  motorSpeed = constrain(speed, 0, 100);
}

/**
 * Drive primitive for 4-wheel differential-drive chassis.
 * @param left  PWM drive signal for left motors (-255 to 255)
 * @param right PWM drive signal for right motors (-255 to 255)
 */
void drive(int left, int right) {
  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // Apply per-wheel PWM trim compensation at the MCU output stage
  int leftPwm  = constrain(abs(left) + leftTrim, 0, 255);
  int rightPwm = constrain(abs(right) + rightTrim, 0, 255);

  digitalWrite(L_DIR, left >= 0 ? HIGH : LOW);
  analogWrite(L_PWM, leftPwm);

  digitalWrite(R_DIR, right >= 0 ? HIGH : LOW);
  analogWrite(R_PWM, rightPwm);
}

/**
 * Immediately stops all drive motors.
 */
void stopMotors() {
  drive(0, 0);
}

/**
 * Adjusts Left & Right motor trim values (-50 to +50) to correct pulling.
 */
void setWheelTrim(int left, int right) {
  leftTrim  = constrain(left, -50, 50);
  rightTrim = constrain(right, -50, 50);
}

// ============================================================================
// SOFTWARE SERVO CONTROL (Zephyr / RouterBridge Compatible)
// ============================================================================

/**
 * Maps servo angle (0-180 deg) to pulse width in microseconds (1000-2000 us).
 */
static int angleToPulseUs(int angle) {
  angle = constrain(angle, 0, 180);
  return 1000 + (int)((long)angle * 1000 / 180);
}

/**
 * Transmits a single software PWM pulse on a specified pin.
 */
static void pulsePin(int pin, int pulseUs) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(pin, LOW);
  delay(SERVO_PERIOD_MS - (pulseUs / 1000));
}

/**
 * Sets a specific robotic arm servo pin to a target angle (0-180 deg).
 * @param pin   Target servo pin (6, 9, 10, or 11)
 * @param angle Target angle in degrees (0 - 180)
 */
void setServo(int pin, int angle) {
  angle = constrain(angle, 0, 180);
  int pw = angleToPulseUs(angle);

  for (int i = 0; i < 4; i++) {
    if (SERVO_PINS[i] == pin) {
      servoAngles[i] = angle;
      for (int p = 0; p < NUM_PULSES; p++) {
        pulsePin(pin, pw);
      }
      return;
    }
  }
}

// ============================================================================
// AUXILIARY ACTUATOR CONTROLS (RELAYS & LASERS)
// ============================================================================

/**
 * Controls herbicide spray relays (Relay 1: Pin 7, Relay 2: Pin 8).
 */
void setRelay(int pin, int state) {
  for (int i = 0; i < 2; i++) {
    if (RELAY_PINS[i] == pin) {
      digitalWrite(RELAY_PINS[i], state ? HIGH : LOW);
      return;
    }
  }
}

/**
 * Controls visual targeting lasers (Laser 1: Pin 12, Laser 2: Pin 13).
 */
void setLaser(int pin, int state) {
  for (int i = 0; i < 2; i++) {
    if (LASER_PINS[i] == pin) {
      digitalWrite(LASER_PINS[i], state ? HIGH : LOW);
      return;
    }
  }
}

// ============================================================================
// MAIN ARDUINO SETUP & ROUTERBRIDGE INITIALIZATION
// ============================================================================

void setup() {
  // Initialize Motor Driver GPIOs
  pinMode(L_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  // Initialize Servo GPIOs and set to calibrated Home Position (90°)
  for (int i = 0; i < 4; i++) {
    pinMode(SERVO_PINS[i], OUTPUT);
    digitalWrite(SERVO_PINS[i], LOW);

    int pw = angleToPulseUs(90);
    for (int p = 0; p < 20; p++) {
      pulsePin(SERVO_PINS[i], pw);
    }
  }

  // Initialize Relays and Lasers OFF
  for (int i = 0; i < 2; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);

    pinMode(LASER_PINS[i], OUTPUT);
    digitalWrite(LASER_PINS[i], LOW);
  }

  // Begin RouterBridge communication link with Python Linux side (MPU)
  Bridge.begin();

  // Register safe RPC calls for remote Linux execution
  Bridge.provide_safe("drive", drive);
  Bridge.provide_safe("stop", stopMotors);
  Bridge.provide_safe("setServo", setServo);
  Bridge.provide_safe("setRelay", setRelay);
  Bridge.provide_safe("setLaser", setLaser);
  Bridge.provide_safe("setWheelTrim", setWheelTrim);

  // Ensure motors start in STOP state
  stopMotors();
}

/**
 * Main execution loop.
 * Note: RPC function calls are dispatched asynchronously via RouterBridge.
 */
void loop() {
  // Handled asynchronously by RouterBridge
}
