//importing libraries for the gyroscope
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;

//declaring necessary variables
float x_angle;
float Kp = 0.8000;
float Ki = 0.0190;
float Kd = 0.0010;
int enA = 3;
int enB = 9;
int in1 = 4;
int in2 = 5;
int in3 = 6;
int in4 = 7;
float actual = 0.0000;
float desired = 0.0000;
float error = 0.0000;
float lasterror=0.0000;
float delta_error = 0.0000;
float errorsum = 0.0000;
float currenttime;
float lasttime;
float delta_T;
int directionA;
int directionB;
float speedpwmA;
float speedpwmB;
float getspeedpwmA;
float speedconspwmA;

void setup() {
  // put your setup code here, to run once:
  actual = get_x_angle();  
  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  actual = get_x_angle();
  error = desired - actual;
  delta_error = error - lasterror;
  currenttime = millis();
  lasterror = error;
  //delta_T = float(currenttime-lasttime)/1000.0000;
  delta_T = 1.0000;
  lasttime = currenttime;
  errorsum = errorsum + (error*delta_T);
  //255/Ki so that errorsum*Ki is never above 255
  errorsum = constrain(errorsum, (-255/Ki), (255/Ki));
  getspeedpwmA = error*Kp + Ki*errorsum + Kd*delta_error; 
  //when getspeedpwmA>0, move backward based on callibration
  if (getspeedpwmA>0){
    directionA = 0;
    directionB = 0;
  } else{
    directionA = 1;
    directionB = 1;
  }
  speedconspwmA = constrain(abs(getspeedpwmA), 93, 255);
  speedpwmA = speedconspwmA;
  speedpwmB = speedpwmA;
  robot_balance(directionA, directionB, speedpwmA, speedpwmB);

}

float get_x_angle(){
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	Serial.print("X : ");
	Serial.print(mpu.getAngleX());
  return mpu.getAngleX();
  timer = millis();  

  }
}

void robot_balance(int dirA, int dirB, int pwmA, int pwmB) {
  analogWrite(enA, pwmA);
  analogWrite(enB, pwmB);
  if (dirA == 1) {
    //forward direction 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    //backwrd direction 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  if (dirB == 1) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}
