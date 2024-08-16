#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Declare the PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  122
#define SERVOMAX  615

#define h1 0
#define h2 1
#define h3 2
#define h4 3

#define k1 4
#define k2 5
#define k3 6
#define k4 7

#define f1 8
#define f2 9
#define f3 10
#define f4 11

int pos[4][3]; // 2D array for goal joint positions
int currPos[4][3]; // 2D array for current joint positions

int ang(int ang) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Angle: "); Serial.print(ang);
    Serial.print(" Pulse: "); Serial.println(pulse);
    return pulse;
}

void jang(int n) {
    int hGoal = pos[n][0];
    int kGoal = pos[n][1];
    int fGoal = pos[n][2];

    Serial.print("Leg "); Serial.print(n); Serial.println(" moving:");
    for (int step = 0; step <= 100; step++) {
        int hCurrent = currPos[n][0] + (hGoal - currPos[n][0]) * step / 100;
        int kCurrent = currPos[n][1] + (kGoal - currPos[n][1]) * step / 100;
        int fCurrent = currPos[n][2] + (fGoal - currPos[n][2]) * step / 100;

        pwm.setPWM(h1 + n, 0, ang(hCurrent));
        pwm.setPWM(k1 + n, 0, ang(kCurrent));
        pwm.setPWM(f1 + n, 0, ang(fCurrent));

        delay(10); // Adjust for smoother or faster transitions
    }

    // Update current positions to match goal positions
    currPos[n][0] = hGoal;
    currPos[n][1] = kGoal;
    currPos[n][2] = fGoal;
}

void hello() {
    for (int i = 0; i < 3; i++) {
        pos[1][0] = 0; // move left leg forward to improve stability

        pos[0][0] = 180; pos[0][2] = 0;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }

        pos[0][0] = 160; pos[0][2] = 0;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
    }
}

void dumforward(int steps) {
    for (int i = 0; i < steps; i++) {
        // Step 1: Move the first pair of legs forward (front right and back left)
        pos[0][0] = 135; pos[0][2] = 90;  // Front right leg
        pos[2][0] = 135; pos[2][2] = 90;  // Back left leg
        pos[1][0] = 90; pos[1][2] = 180;  // Front left leg remains stationary
        pos[3][0] = 90; pos[3][2] = 180;  // Back right leg remains stationary

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);

        // Step 2: Move the second pair of legs forward (front left and back right)
        pos[0][0] = 90; pos[0][2] = 180;  // Front right leg remains stationary
        pos[2][0] = 90; pos[2][2] = 0;  // Back left leg remains stationary
        pos[1][0] = 135; pos[1][2] = 90;  // Front left leg
        pos[3][0] = 135; pos[3][2] = 90;  // Back right leg

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);

        // Step 3: Reset legs to prepare for the next cycle
        pos[0][0] = 90; pos[0][2] = 180;  // Front right leg
        pos[2][0] = 90; pos[2][2] = 0;  // Back left leg
        pos[1][0] = 90; pos[1][2] = 180;  // Front left leg
        pos[3][0] = 90; pos[3][2] = 180;  // Back right leg

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);
    }
}

void forward(int steps) {
    for (int i = 0; i < steps; i++) {
        pos[0][0] = 90; pos[0][2] = 180; //1
        pos[1][0] = 45; pos[1][2] = 0;
        pos[2][0] = 135; pos[2][2] = 180;
        pos[3][0] = 45; pos[3][2] = 90;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);

        pos[0][0] = 90; pos[0][2] = 90; //2
        pos[1][0] = 45; pos[1][2] = 0;
        pos[2][0] = 135; pos[2][2] = 180;
        pos[3][0] = 90; pos[3][2] = 180;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);

        pos[0][0] = 135; pos[0][2] = 180; //3
        pos[1][0] = 90; pos[1][2] = 0;
        pos[2][0] = 153; pos[2][2] = 180;
        pos[3][0] = 45; pos[3][2] = 180;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);

        pos[0][0] = 135; pos[0][2] = 180; //4
        pos[1][0] = 90; pos[1][2] = 0;
        pos[2][0] = 153; pos[2][2] = 90;
        pos[3][0] = 45; pos[3][2] = 180;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);

        pos[0][0] = 135; pos[0][2] = 180; //5
        pos[1][0] = 90; pos[1][2] = 90;
        pos[2][0] = 90; pos[2][2] = 180;
        pos[3][0] = 45; pos[3][2] = 180;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);

        pos[0][0] = 90; pos[0][2] = 180; //6
        pos[1][0] = 45; pos[1][2] = 0;
        pos[2][0] = 135; pos[2][2] = 180;
        pos[3][0] = 27; pos[3][2] = 180;

        for (int i = 0; i < 4; i++) {
            jang(i);
        }
        delay(500);
    }
}

void setup() {
    Serial.begin(9600);

    // Initialize I2C with custom SDA and SCL pins
    Wire.begin(21, 22);

    pwm.begin();
    pwm.setPWMFreq(60);

    int initialPosition = 90;
    for (int j = 0; j < 4; j++) {
        pos[j][0] = initialPosition;
        pos[j][1] = initialPosition;
        pos[j][2] = initialPosition;

        currPos[j][0] = initialPosition;
        currPos[j][1] = initialPosition;
        currPos[j][2] = initialPosition;

        delay(500);
    }

    for (int i = 0; i < 4; i++) {
        jang(i);
    }
    delay(1000);
}
