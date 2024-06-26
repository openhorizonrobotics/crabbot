#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo constants
#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz

// Define the joint coordinates
int jc[4][3][3] = {
    {{85, 45, 20}, {165, 45, 20}, {285, 45, 20}},
    {{-85, 45, 20}, {-165, 45, 20}, {-285, 45, 20}},
    {{-85, -45, 20}, {-165, -45, 20}, {-285, -45, 20}},
    {{85, -45, 20}, {165, -45, 20}, {285, -45, 20}}
};

// Function to convert degrees to radians
double degToRad(double degrees) {
    return degrees * PI / 180.0;
}

// Function to convert angle to PWM pulse length
int angleToPulse(int angle) {
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Function to set servo angle
void setServoAngle(int servoNum, int angle) {
    int pulse = angleToPulse(angle);
    pwm.setPWM(servoNum, 0, pulse);
}

// Function to initialize servos
void initServos() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
}

// Function to apply the rotation matrices to the joint coordinates
void applyRotation(int leg, int joint, double angle) {
    double x = jc[leg][joint][0];
    double y = jc[leg][joint][1];
    double z = jc[leg][joint][2];
    
    if (z == 0) { // Rotation around Z axis
        rotateZ(angle, &x, &y);
    } else { // Rotation around Y axis
        rotateY(angle, &x, &z);
    }
    
    jc[leg][joint][0] = (int)round(x);
    jc[leg][joint][1] = (int)round(y);
    jc[leg][joint][2] = (int)round(z);

    // Set the servo angle based on the calculated rotation
    setServoAngle(leg * 3 + joint, angle); // Assuming servos are connected in sequence for each joint
}

// Rotation matrix around Z axis
void rotateZ(double angle, double *x, double *y) {
    double rad = degToRad(angle);
    double cosA = cos(rad);
    double sinA = sin(rad);
    double newX = *x * cosA - *y * sinA;
    double newY = *x * sinA + *y * cosA;
    *x = newX;
    *y = newY;
}

// Rotation matrix around Y axis
void rotateY(double angle, double *x, double *z) {
    double rad = degToRad(angle);
    double cosA = cos(rad);
    double sinA = sin(rad);
    double newX = *x * cosA + *z * sinA;
    double newZ = -*x * sinA + *z * cosA;
    *x = newX;
    *z = newZ;
}

// Function to print joint coordinates
void printJointCoordinates() {
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            Serial.printf("Leg %d, Joint %d: [%d, %d, %d]\n", leg, joint, jc[leg][joint][0], jc[leg][joint][1], jc[leg][joint][2]);
        }
    }
    Serial.println();
}

// Function to simulate a simple walking gait
void walkGait() {
    // Example gait: move each leg sequentially
    double stepAngle = 15.0; // Example angle for each step

    // Move each leg forward and backward
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            // Move forward
            applyRotation(leg, joint, stepAngle);
        }
        printJointCoordinates(); // Print the coordinates after moving forward

        delay(1000); // Wait for 1 second

        for (int joint = 0; joint < 3; joint++) {
            // Move backward
            applyRotation(leg, joint, -stepAngle);
        }
        printJointCoordinates(); // Print the coordinates after moving backward

        delay(1000); // Wait for 1 second
    }
}

void setup() {
    Serial.begin(115200);
    initServos();

    // Print initial joint coordinates
    Serial.println("Initial joint coordinates:");
    printJointCoordinates();

    // Simulate walking gait
    walkGait();
}

void loop() {
    // Add additional gait control logic here
}
