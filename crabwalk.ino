#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 
#define SERVOMAX  600 

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

int ang(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}

void setup() {
  Serial.begin(9600);
  Serial.println("good doggi");

  pwm.begin();
  pwm.setPWMFreq(60); 

    pwm.setPWM(0, 0, ang(90));
    pwm.setPWM(1, 0, ang(90));
    pwm.setPWM(2, 0, ang(90));
    pwm.setPWM(3, 0, ang(90));
    pwm.setPWM(4, 0, ang(90));
    pwm.setPWM(5, 0, ang(90));
    pwm.setPWM(6, 0, ang(90));
    pwm.setPWM(7, 0, ang(90));
    pwm.setPWM(8, 0, ang(90));
    pwm.setPWM(9, 0, ang(90));
    pwm.setPWM(10, 0, ang(90));
    pwm.setPWM(11, 0, ang(90));
    delay(1000);

}

void loop() {

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

    /*pwm.setPWM(0, 0, 377);
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
