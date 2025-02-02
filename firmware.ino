#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// Create the PCA9685 driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ----- Servo Parameters -----
// The PCA9685 uses a 12-bit resolution (0–4095). When running at 60 Hz,
// one period is ~16.67ms. With pulse widths between 500us and 2500us,
// the corresponding counts are approximately:
const uint16_t SERVOMIN = 123; // ~500us pulse
const uint16_t SERVOMAX = 614; // ~2500us pulse

// Helper function to set a servo (by channel) to an angle in degrees (0-180)
void setServo(uint8_t channel, float angle) {
  // Map the angle to a pulse width (in PCA9685 counts)
  uint16_t pulse = SERVOMIN + (uint16_t)((angle / 180.0) * (SERVOMAX - SERVOMIN));
  pwm.setPWM(channel, 0, pulse);
}

// ----- Global Variables and Constants for IK -----
unsigned long interval = 1;
unsigned long previousMillis = 0;

float Pi = 3.141592653589793;
float G_C = 50.00; // ground clearance
float Y_Offset;
float D;
float d;
float R;
float Coxa = 36.00;
float Femur = 50.00;
float Tibia = 85.00;

float Alpha_1;
float Alpha_2;
float Theta_1;
float Theta_2;
float Theta_3;

float X_1, Y_1, Z_1;
float X_2, Y_2, Z_2;
float X_3, Y_3, Z_3;
float X_4, Y_4, Z_4;

// ----- Inverse Kinematics Functions -----
void Rumus_IK_1() {
  if (Z_1 > 0.00) {
    D = sqrt(pow(X_1, 2) + pow(Z_1, 2));
    Theta_1 = (atan(X_1 / Z_1)) * (180.00 / Pi);
    d = D - Coxa;
    Y_Offset = G_C - Y_1;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_1 == 0.00) {
    D = sqrt(pow(X_1, 2) + pow(Z_1, 2));
    Theta_1 = 90.00;
    d = D - Coxa;
    Y_Offset = G_C - Y_1;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_1 < 0.00) {
    D = sqrt(pow(X_1, 2) + pow(Z_1, 2));
    Theta_1 = 90.00 + (90.00 - fabs((atan(X_1 / Z_1)) * (180.00 / Pi)));
    d = D - Coxa;
    Y_Offset = G_C - Y_1;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
}

void Rumus_IK_2() {
  if (Z_2 > 0.00) {
    D = sqrt(pow(X_2, 2) + pow(Z_2, 2));
    Theta_1 = (atan(X_2 / Z_2)) * (180.00 / Pi);
    d = D - Coxa;
    Y_Offset = G_C - Y_2;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_2 == 0.00) {
    D = sqrt(pow(X_2, 2) + pow(Z_2, 2));
    Theta_1 = 90.00;
    d = D - Coxa;
    Y_Offset = G_C - Y_2;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_2 < 0.00) {
    D = sqrt(pow(X_2, 2) + pow(Z_2, 2));
    Theta_1 = 90.00 + (90.00 - fabs((atan(X_2 / Z_2)) * (180.00 / Pi)));
    d = D - Coxa;
    Y_Offset = G_C - Y_2;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
}

void Rumus_IK_3() {
  if (Z_3 > 0.00) {
    D = sqrt(pow(X_3, 2) + pow(Z_3, 2));
    Theta_1 = (atan(X_3 / Z_3)) * (180.00 / Pi);
    d = D - Coxa;
    Y_Offset = G_C - Y_3;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_3 == 0.00) {
    D = sqrt(pow(X_3, 2) + pow(Z_3, 2));
    Theta_1 = 90.00;
    d = D - Coxa;
    Y_Offset = G_C - Y_3;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_3 < 0.00) {
    D = sqrt(pow(X_3, 2) + pow(Z_3, 2));
    Theta_1 = 90.00 + (90.00 - fabs((atan(X_3 / Z_3)) * (180.00 / Pi)));
    d = D - Coxa;
    Y_Offset = G_C - Y_3;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
}

void Rumus_IK_4() {
  if (Z_4 > 0.00) {
    D = sqrt(pow(X_4, 2) + pow(Z_4, 2));
    Theta_1 = (atan(X_4 / Z_4)) * (180.00 / Pi);
    d = D - Coxa;
    Y_Offset = G_C - Y_4;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_4 == 0.00) {
    D = sqrt(pow(X_4, 2) + pow(Z_4, 2));
    Theta_1 = 90.00;
    d = D - Coxa;
    Y_Offset = G_C - Y_4;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
  else if (Z_4 < 0.00) {
    D = sqrt(pow(X_4, 2) + pow(Z_4, 2));
    Theta_1 = 90.00 + (90.00 - fabs((atan(X_4 / Z_4)) * (180.00 / Pi)));
    d = D - Coxa;
    Y_Offset = G_C - Y_4;
    R = sqrt(pow(d, 2) + pow(Y_Offset, 2));
    Alpha_1 = (acos(Y_Offset / R)) * (180.00 / Pi);
    Alpha_2 = (acos((pow(Femur, 2) + pow(R, 2) - pow(Tibia, 2)) / (2 * Femur * R))) * (180.00 / Pi);
    Theta_2 = Alpha_1 + Alpha_2;
    Theta_3 = (acos((pow(Femur, 2) + pow(Tibia, 2) - pow(R, 2)) / (2 * Femur * Tibia))) * (180.00 / Pi);
  }
}

// ----- Setup and Main Loop -----
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);  // Set frequency to 60 Hz

  // ---- Initial (Stance) Position ----
  // Leg 1 (Channels: Coxa_1=0, Femur_1=1, Tibia_1=2)
  {
    X_1 = 70.00;
    Y_1 = 0.00;
    Z_1 = 40.00;
    Rumus_IK_1();
    setServo(0, Theta_1);
    setServo(1, Theta_2);
    setServo(2, Theta_3);
  }
  // Leg 2 (Channels: Coxa_2=3, Femur_2=4, Tibia_2=5)
  {
    X_2 = 70.00;
    Y_2 = 0.00;
    Z_2 = -40.00;
    Rumus_IK_2();
    setServo(3, Theta_1);
    setServo(4, 180.00 - Theta_2);
    setServo(5, 180.00 - Theta_3);
  }
  // Leg 3 (Channels: Coxa_3=6, Femur_3=7, Tibia_3=8)
  {
    X_3 = 70.00;
    Y_3 = 0.00;
    Z_3 = 0.00;
    Rumus_IK_3();
    setServo(6, 180.00 - Theta_1);
    setServo(7, 180.00 - Theta_2);
    setServo(8, 180.00 - Theta_3);
  }
  // Leg 4 (Channels: Coxa_4=9, Femur_4=10, Tibia_4=11)
  {
    X_4 = 70.00;
    Y_4 = 0.00;
    Z_4 = 0.00;
    Rumus_IK_4();
    setServo(9, 180.00 - Theta_1);
    setServo(10, Theta_2);
    setServo(11, Theta_3);
  }
}

void loop() {
  // Use millis() to pace the motions
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    
    // ---- Leg 3 Movement ----
    // 1. Lift leg 3 (increasing Y)
    Y_3 = 0.00;
    do {
      X_3 = 70.00;
      Z_3 = 0.00;
      Y_3 += 0.2;
      Rumus_IK_3();
      setServo(6, 180.00 - Theta_1);
      setServo(7, 180.00
