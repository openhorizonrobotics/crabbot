//This code is still pending testing, and hence may not work

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 //min and max pulse length for the servos (change based on your servo model)
#define SERVOMAX  600 

//defining 12 joints from 0 to 12
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
//makes code more readable

int pos[4][3]; //2d array that remembers the current position of all the joints

int ang(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}

//funtion takes the leg number as input(0-3) and changes joint positions based on pos array
void jang(int n) {
    int h = pos[i][0],
        k = pos[i][1],
        f = pos[i][2];

    switch(n) {
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
            Serial.print("invalid leg");
    }
}


void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); 

  for (int i=0;i<4;i++)
  {
    pos[i] = {90, 90, 90}
    jang(i);
  }
  delay(1000);

}

void loop() {


jang(0,135,45,135);
jang(1,45 ,45 ,135);
jang(2,45 ,135,45);
jang(3,135,135 ,45);

delay(1000);

jang(3,112,90,45);
delay(100);
jang(3,90,135,45);
delay(5000);

//Old code, useful for troubleshooting
/* 
pwm.setPWM(h1, 0,ang(45) );
pwm.setPWM(h2, 0,ang(135) );
pwm.setPWM(h3, 0,ang(45) );
pwm.setPWM(h4, 0,ang(135) );
delay(1000);


pwm.setPWM(k1, 0,ang(45) );
pwm.setPWM(f1, 0,ang(0) );
delay(500);
pwm.setPWM(h1, 0,ang(45) );
delay(500);  
pwm.setPWM(k1, 0,ang(90) );
pwm.setPWM(f1, 0,ang(0) );    
delay(500);
/*
    
pwm.setPWM(5, 0, 100);
pwm.setPWM(1, 0, 377);
delay(100);
pwm.setPWM(5, 0, 285);
delay(100);

pwm.setPWM(6, 0, 470);
pwm.setPWM(2, 0, 192);
delay(100);
pwm.setPWM(6, 0, 285);
delay(100);
    
pwm.setPWM(7, 0, 470);
pwm.setPWM(3, 0, 285);
delay(100);
pwm.setPWM(7, 0, 285);
delay(100);

pwm.setPWM(4, 0, 100);
pwm.setPWM(0, 0, 285);
delay(100);
pwm.setPWM(4, 0, 285);
delay(100);

/*
pwm.setPWM(0, 0, 377);
pwm.setPWM(1, 0, 377);
pwm.setPWM(2, 0, 192);
pwm.setPWM(3, 0, 192);
delay(1000);
/*
pwm.setPWM(0, 0, 192);
pwm.setPWM(1, 0, 192);
pwm.setPWM(2, 0, 192);
pwm.setPWM(3, 0, 192);
pwm.setPWM(4, 0, 285);
pwm.setPWM(5, 0, 285);
pwm.setPWM(6, 0, 285);
pwm.setPWM(7, 0, 285);
pwm.setPWM(8, 0, 100);
pwm.setPWM(9, 0, 100);
pwm.setPWM(10, 0, 470);
pwm.setPWM(11, 0, 470);
delay(1000);
*/
}
