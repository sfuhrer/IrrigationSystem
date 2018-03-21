#include <SoftwareSerial.h>


//blalballlabaslbfds

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

#include <Time.h>
#include <TimeAlarms.h>


const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
bool startup = true;

//servo setup
int servo_left_pos = 0;  //variable controlling position
Servo server_left;  // servo object of left garden
Servo servo_switch;


// pump setup
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *left_pump = AFMS.getMotor(1);

//moisture setup
int moistureSensorPin = A0;    // select the input pin for the sensor

void setup() {

  Serial.begin(9600);

  setTime(16,16,0,7,7,17); // set time to Saturday 8:29:00am Jan 1 2017
  // create the alarms 
  Alarm.alarmRepeat(7,00,0, water_everything);  // 7:00 every day it is executing 'water_everything'
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  server_left.attach(10);  // attaches the servo on pin 10 to the servo object
  servo_switch.attach(9);
  servo_switch.write(0);
  Serial.println("set to 0");
  delay(3000);
  servo_switch.write(180);
  Serial.println("set to 0");
  delay(3000);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  Serial.println("release it");
  left_pump->setSpeed(0);
  left_pump->run(RELEASE);
 
}

int get_moisture(){
  return analogRead(moistureSensorPin);
}

void water_everything(){

  Serial.println("starting watering function");
  //switch on servo
  servo_switch.write(0);
  Serial.println("set to 0");
  delay(3000); // somehow we need those delays, otherwise it does not work....fuuu

  Serial.println("setting speed");
  left_pump->setSpeed(255);
  Serial.println("setting forward");
  left_pump->run(BACKWARD);
  
  /*wait until it is more moist
  for (;;){
    if (true || current_moisture - get_moisture() > 300)
      break;
    Serial.println(get_moisture());
  }
*/
    unsigned long start = millis();
   //perform sweep
   for(;;){
    Serial.println(millis()-start); 
    if (millis()-start > 500000)
      break;
    Serial.println("right sweep");
    for (int i = 40; i < 150; i++){
      server_left.write(i);
      
      delay(100);
    }
    delay(2000);
    Serial.println("left sweep");
    for (int i = 150; i >40; i--){
      server_left.write(i);
      
      delay(100);
    }
   }
   delay(4000);
  //stop pump
  left_pump->run(RELEASE);

  //switch off servo
  servo_switch.write(180);
  Serial.println("set to 0");
  delay(3000);
  return;
}

void loop() {
  if (startup){
    startup = false;
    water_everything();
  }
 /* 
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed
  if (buttonState == LOW) {
    //water stuff
    Serial.println("start watering it");
    water_everything();
  }
    
  for(;;){
    delay(100);
  }
*/
digitalClockDisplay();
  Alarm.delay(1000); // wait one second between clock display
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println(); 
}

void printDigits(int digits)
{
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


