
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

int pos[4][3]; // 2D array for joint positions


int ang(int ang) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    return pulse;
}

void jang(int n) {
    int h = pos[n][0];
    int k = pos[n][1];
    int f = pos[n][2];

    switch (n) {
        case 0:
            pwm.setPWM(h1, 0, ang(h));
            pwm.setPWM(k1, 0, ang(k));
            pwm.setPWM(f1, 0, ang(f));
            break;
        case 1:
            pwm.setPWM(h2, 0, ang(h));
            pwm.setPWM(k2, 0, ang(k));
            pwm.setPWM(f2, 0, ang(f));
            break;
        case 2:
            pwm.setPWM(h3, 0, ang(h));
            pwm.setPWM(k3, 0, ang(k));
            pwm.setPWM(f3, 0, ang(f));
            break;
        case 3:
            pwm.setPWM(h4, 0, ang(h));
            pwm.setPWM(k4, 0, ang(k));
            pwm.setPWM(f4, 0, ang(f));
            break;
        default:
            Serial.println("Invalid leg");
    }
}

void hello(){

  //makes the bot wave hello
  
    for (int i = 0; i < 3; i++) {

    pos[1][0] = 0; //move left leg forward to improve stability
    
    pos[0][0] = 180;  pos[0][2] = 0;

    for (int i = 0; i < 4; i++) {
        jang(i);
    }
    pos[0][0] = 160;  pos[0][2] = 0;

    for (int i = 0; i < 4; i++) {
        jang(i);
    };
 }
}

void rave(){
  pos[0][0] = 90;
  pos[1][0] = 90;
  pos[2][0] = 90;
  pos[3][0] = 90;
  jang(1);jang(2);jang(3);jang(4);

  pos[0][2] = 180;
  pos[1][2] = 0;
  pos[2][2] = 180;
  pos[3][2] = 180;
  jang(1);jang(2);jang(3);jang(4);

}

void x(){

    pos[0][0] = 135;  pos[0][2] = 0;
    pos[1][0] = 45;  pos[1][2] = 180;
    pos[2][0] = 45;  pos[2][2] = 180;
    pos[3][0] = 135;  pos[3][2] = 180;
    for (int i = 0; i < 4; i++) {
        jang(i);
    }
}

void dumforward(int steps){
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

        // Step 3: Reset legs to prepare for next cycle
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

void wait(){

    pos[0][0] = 135;  pos[0][2] = 180;
    pos[1][0] = 45;   pos[1][2] = 0;
    pos[2][0] = 135;  pos[2][2] = 180;
    pos[3][0] = 45;   pos[3][2] = 180;
    for (int i = 0; i < 4; i++)
        jang(i);

    pos[1][0] = 90;
        jang(1);

    pos[1][0] = 0;  pos[1][2] = 180;
    for (int i = 0; i < 4; i++) 
        jang(i);

}


void forward(int steps){

  for (int i=0;i<=steps;i++){
    pos[0][0] = 90; pos[0][2] = 180; //1
    pos[1][0] = 45; pos[1][2] = 0;
    pos[2][0] = 135; pos[2][2] = 180;
    pos[3][0] = 45; pos[3][2] = 90;

    for (int i = 0; i < 4; i++) {
        jang(i);}
      
    

    pos[0][0] = 90; pos[0][2] = 90; //2
    pos[1][0] = 45; pos[1][2] = 0;
    pos[2][0] = 135; pos[2][2] = 180;
    pos[3][0] = 90; pos[3][2] = 180;    

    for (int i = 0; i < 4; i++) {
        jang(i);}


    pos[0][0] = 135; pos[0][2] = 180; //3
    pos[1][0] = 90; pos[1][2] = 0;
    pos[2][0] = 153; pos[2][2] = 180;
    pos[3][0] = 45; pos[3][2] = 180;    

    for (int i = 0; i < 4; i++) {
        jang(i);}

    
    pos[0][0] = 135; pos[0][2] = 180; //4
    pos[1][0] = 90; pos[1][2] = 0;
    pos[2][0] = 153; pos[2][2] = 90;
    pos[3][0] = 45; pos[3][2] = 180;    

    for (int i = 0; i < 4; i++) {
        jang(i);}


    pos[0][0] = 135; pos[0][2] = 180; //5
    pos[1][0] = 90; pos[1][2] = 90;
    pos[2][0] = 90; pos[2][2] = 180;
    pos[3][0] = 45; pos[3][2] = 180;    

    for (int i = 0; i < 4; i++) {
        jang(i);}

    
    pos[0][0] = 90; pos[0][2] = 180; //6
    pos[1][0] = 45; pos[1][2] = 0;
    pos[2][0] = 135; pos[2][2] = 180;
    pos[3][0] = 27; pos[3][2] = 180;    

    for (int i = 0; i < 4; i++) {
        jang(i);}

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
        delay(500);
    }

    for (int i = 0; i < 4; i++) {
        jang(i);
    }
    delay(1000);
}


void loop() {

hello();
delay(1000);
}
