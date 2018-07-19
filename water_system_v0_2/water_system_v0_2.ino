#include <SoftwareSerial.h>


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

#include <Time.h>
#include <TimeAlarms.h>

#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

//const int buttonPin = 2;     // the number of the pushbutton pin
//int buttonState = 0;         // variable for reading the pushbutton status
bool startup = true;

//servo setup
int servo_left_pos = 0;  //variable controlling position
Servo sweepServo;  // servo object of sweeping servo
Servo powerSwitch;  // servo object of power switch


// pump setup
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *left_pump = AFMS.getMotor(1);
Adafruit_DCMotor *second_pump = AFMS.getMotor(2); // 2nd pump

Adafruit_DCMotor *servo_power = AFMS.getMotor(3);

//moisture setup
//int moistureSensorPin = A0;    // select the input pin for the sensor

void setup() {

  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
  
  // setTime(14, 27, 00, 14, 12, 2015);  // set the current time to 14:27:00, December 14th, 2015
  setTime(0,0,0,1,1,2017); // set time to 00:00.00 h 1.1.2017
  printTime();
  
  // initialize the pushbutton pin as an input:
  //pinMode(buttonPin, INPUT);
  
  sweepServo.attach(10);  // attaches the servo on pin 10 to the servo object
  powerSwitch.attach(9);
  
  powerSwitch.write(0);   //turn on power switch
  Serial.println("turn ON power to Adafruit board");
  delay(3000);
  powerSwitch.write(180);
  Serial.println("turn OFF power to Adafruit board");
  delay(3000);
  
  AFMS.begin();  // create with the default frequency 1.6KHz

  left_pump->setSpeed(0);
  left_pump->run(RELEASE);
  
  second_pump->setSpeed(0);
  second_pump->run(RELEASE); 

  servo_power->setSpeed(0);
  servo_power->run(RELEASE);

   // create the alarms 
  Alarm.alarmRepeat(15,0,0, water_alarm);  //12h after start up/button press it is executing 'water_everything'

  uint32_t currentFrequency;
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
 
}

/*
int get_moisture(){
  return analogRead(moistureSensorPin);
}
*/

void water_everything(int duration, int pressure){

  Serial.print("Starting watering function. Turn on board. Time: "); printTime();
  Serial.print("Duration: "); Serial.println(duration);
  Serial.print("Prssure: "); Serial.println(pressure); 
  powerSwitch.write(0); //turn on power
  delay(3000); // somehow we need those delays, otherwise it does not work....fuuu

  left_pump->setSpeed(pressure);
  left_pump->run(FORWARD); 

  servo_power->setSpeed(170);
  servo_power->run(FORWARD);
  
  //measCurrent();
  
  /*wait until it is more moist
  for (;;){
    if (true || current_moisture - get_moisture() > 300)
      break;
    Serial.println(get_moisture());
  }
*/
    unsigned long start = millis();
    unsigned long runtime = 0;
    unsigned long dur = duration;
    unsigned long endMilis = dur * 1000;
    
   //perform sweep
   //for(;;){

   
   while(runtime <=  endMilis) {
    Serial.print("dur: "); Serial.println(dur); 
    Serial.print("endMilis: "); Serial.println(endMilis); 
    Serial.print("runtime: "); Serial.println(runtime); 
    
    Serial.println("right sweep");
    for (int i = 40; i < 150; i++){
      sweepServo.write(i);      
      delay(100);
    }
    delay(2000);
    Serial.println("left sweep");
    for (int i = 150; i >40; i--){
      sweepServo.write(i);    
      delay(100);
    }
    runtime = millis() - start;
   }
   //delay(4000);
  //stop pump
  left_pump->run(RELEASE);
  //measCurrent();

  //switch off servo
  powerSwitch.write(180);
  //servo_power->setSpeed(0);
  Serial.println("End of water_everything. Turn OFF Adafruit board. Time: "); printTime();
  delay(3000);
  return;
}

void waterSecondary(int duration){
  unsigned long start2 = millis();
  unsigned long runtime2 = 0;
  
  second_pump->setSpeed(30);
  second_pump->run(FORWARD);
  Serial.println("Water secondary.");

  while ( runtime2 < duration * 1000) {
    runtime2 = millis() - start2;
  }

  //delay(4000);
  //stop pump
  second_pump->run(RELEASE);
  Serial.print("Finished water secondary. Time:"); printTime();
  
}


void measCurrent(void)
{

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");

  
}

void water_alarm() {
    water_everything(50,25);  // low pressure
    water_everything(200,255); // high pressure
    waterSecondary(20);
    Serial.println("*********Finished watering. Wait for alarm.***********");
    return;
}



void loop() {
  if (startup){
    startup = false;
    water_everything(10,25);
    water_everything(10,255);
    waterSecondary(3);
    Serial.println("***********Finished start up. Wait for alarm.************");
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
printTime();
 Alarm.delay(60000); // wait 1 min between clock display
}

//******************************************************
//helper functions

void printTime(){
  print2digits(hour());
  Serial.print(":");
  print2digits(minute());
  Serial.print(":");
  print2digits(second());

  Serial.print(" ");

  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());

  Serial.println();
  
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.print('0');
  }
  Serial.print(number);
}



