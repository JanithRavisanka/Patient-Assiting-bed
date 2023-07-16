#define USE_ARDUINO_INTERRUPTS true
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PulseSensorPlayground.h>
#include "DHT.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <uRTCLib.h>
#include <SoftwareSerial.h>
#include <HX711_ADC.h>
#include "VoiceRecognitionV3.h"



//vr module
VR myVR(10, 36);    // 2:RX 3:TX, you can choose your favorite pins.

uint8_t records[7]; // save record
uint8_t buf[64];


#define onRecord    (0)
#define offRecord   (1) 
#define onRecord1    (2)
#define offRecord1   (3)
#define onRecord2   (4)
#define offRecord2   (5)




// Pins fan + Load cell
//DHT pin declared above 
  //Load cell
const int HX711_dout = 25;  // MCU > HX711 dout pin
const int HX711_sck = 23;  // MCU > HX711 sck pin
const int RELAYPIN = 7;    // MCU > LED pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//touch sensors
// const int toggleModePin = 11; 
// const int bedUpPin = 12;
// const int bedDownPin = 13;

const int toggleModePin = 12;
const int bedUpPin = 11;
const int bedDownPin = 46;
bool invalid = false;
unsigned long t = 0;
bool newDataReady = 0;
String toggleMode = "up";
unsigned long touchInterval = 0;

//motor
const int motorPinUp = 27; //up
const int motorPinDown = 29; //down

unsigned long functionStartTime = 0;
unsigned long functionEndTime = 0;


//body temperature sensor
#define bodyTempPin 9
OneWire bTempPin(bodyTempPin);
DallasTemperature bodyTempSensor(&bTempPin);

//heart pulse sensor
PulseSensorPlayground pulseSensor;
const int PulseWire = A2;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED = LED_BUILTIN;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 650;  
int count = 0;
int bpm = 0;
unsigned long lastPulse = 0;
int signal = 0;
unsigned long lastBPM = 0;
int currentBPM = 0;

//DHT sensor
#define DHTPIN 8
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//RTC
uRTCLib rtc(0x68);

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};


//GSM module
SoftwareSerial mySerial(3, 2);


//function to read temperature
void readBodyTemp();

//fuction to read pulse
void readPulse();

//function to read humidity and room temperature
float readDHT();

void createDisplay();

void printTime();

void sendAlerts(String str);

void sendMessage(String str);

int isWeightDetected();

int overHumidity();

int overTemperature();

void printSignature(uint8_t *buf, int len);

void printVR(uint8_t *buf);

void updateSerial();


void setup() {
  //serial connection with NodeMCU
  Serial.begin(115200);
  Serial1.begin(115200);

  //vr module
  myVR.begin(9600);
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
  // pinMode(led, OUTPUT);

  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    // while(1);
  }
  
  if(myVR.load((uint8_t)onRecord) >= 0){
    Serial.println("onRecord loaded");
  }
  
  if(myVR.load((uint8_t)offRecord) >= 0){
    Serial.println("offRecord loaded");
  }
  //mortor
  pinMode(motorPinUp, OUTPUT);
  pinMode(motorPinDown, OUTPUT);


  //touch sensors
  pinMode(toggleModePin, INPUT);
  pinMode(bedUpPin, INPUT);
  pinMode(bedDownPin, INPUT);



  //serial connection for gsm module
  mySerial.begin(9600);

  //body temp sensor
  bodyTempSensor.begin();

  //dht sensor
  dht.begin();

  if (isnan(dht.readHumidity())) {
    Serial.println(F("Failed to read from DHT sensor!"));
    //create infinite while loop
    while (1){
      Serial1.println("Connecting to DHT sensor...");
    }
  }

  //display and rtc
  URTCLIB_WIRE.begin();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, heart);

  //gsm module
  Serial.println("Initializing GSM Module...");
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"+Z94703487817\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  mySerial.print("Message send from Patient Assisting Bed.This is a testing message"); //text content
  updateSerial();
  mySerial.write(26);

  LoadCell.begin();
  float calibrationValue;
  calibrationValue = -115.53;

  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU > HX711 wiring and pin designations");
    while (1);
    Serial.println("LoadCell Startup is complete");
  } else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup is complete");
  }

}


void loop() { 
  

  if(digitalRead(toggleModePin)==HIGH && toggleMode=="up" && (millis() - touchInterval > 1000)){
    touchInterval = millis();
    Serial.println("Toggle " + toggleMode);
    toggleMode = "down";
    digitalWrite(motorPinUp, LOW);
    digitalWrite(motorPinDown, HIGH);
    Serial.println("motor pin up:" + String(digitalRead(motorPinUp))+ ","+"motor pin down:" + String(digitalRead(motorPinDown)));

  }
  else if(digitalRead(toggleModePin)==HIGH && toggleMode=="down" && (millis() - touchInterval > 1000)){
    touchInterval = millis();
    Serial.println("Toggle " + toggleMode);
    toggleMode = "up";
    digitalWrite(motorPinUp, HIGH);
    digitalWrite(motorPinDown, LOW);
    Serial.println("motor pin up:" + String(digitalRead(motorPinUp))+ ","+"motor pin down:" + String(digitalRead(motorPinDown)));
  }else if(digitalRead(bedUpPin) ==HIGH && (millis() - touchInterval > 1000)){
    touchInterval = millis();
    digitalWrite(motorPinUp, LOW);
    digitalWrite(motorPinDown, LOW);
    Serial.println("pin 11: " + String(digitalRead(bedUpPin)));
  }else if(digitalRead(bedUpPin) ==LOW && (millis() - touchInterval > 1000)){
    touchInterval = millis();
    Serial.println("pin 11: " + String(digitalRead(bedUpPin)));
  }



  createDisplay();
  printTime();


  bpm = 0;
  if(bpm>0){
    float humidity = dht.readHumidity();
    float roomTemp = dht.readTemperature();
    float bodyTemp = bodyTempSensor.getTempCByIndex(0);
    currentBPM = bpm;  //show curent bpm in display

    Serial1.println("humidity =" + String(humidity));
    Serial1.println("roomTemp=" + String(roomTemp));
    Serial1.println("bodyTemp=" + String(bodyTemp));
    Serial1.println("bpm=" + String(bpm));
  }

  //only check bpm and body temp critical in 5 min intervals
  if(millis() - lastBPM > 300000){
    checkBPM(bpm);
    checkBodyTemp(bodyTempSensor.getTempCByIndex(0));
  } 

  // bedLiftingFunction(); 

  checkPatientThere();

  //relay + dht to on the fan 
  int isWeight = isWeightDetected();
  int ishumid  = overHumidity();
  int istemp   = overTemperature();
    
  if ((isWeight == 1 && ishumid==1) || (isWeight == 1 && istemp==1)) {
    // Serial.println("both");
    digitalWrite(RELAYPIN, HIGH);
  }

  else {
    // Serial.println("neither");
    digitalWrite(RELAYPIN, LOW);
  }  

  //vr recog
  vrRecog();

}


//read body temperature
void readBodyTemp() {
  bodyTempSensor.requestTemperatures();
  Serial1.println("bTemp=" + String(bodyTempSensor.getTempCByIndex(0)));
  delay(10);
}

//create function to output random hr values without sensor readings
void readPulse() {
  // int pulse = random(60, 100);
  // Serial1.println("pulse = " + String(pulse));
  signal = analogRead(PulseWire);
  if(signal > Threshold && millis() - lastPulse> 300){
    count++;
    lastPulse = millis(); //get the last time a pulse was read
  }
  if(millis() - lastBPM > 60000){
    bpm = count;
    count = 0; //reset count
    lastBPM = millis(); //get the last time a BPM was read
    Serial1.println("bpm=" + String(bpm));
  }
}

float readDHT() {
  // float h = dht.readHumidity();
  // float t = dht.readTemperature();
  //random values for testing
  float h = random(60, 100);
  float t = random(60, 100);

  // Serial1.println("humidity =" + String(h));

  return h,t;
  // Serial1.println("roomTemp=" + String(t));
}


void createDisplay(){
  rtc.refresh();
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(0);
  lcd.setCursor(0, 1);
  lcd.print("Body Temp: " + String(dht.readHumidity()) + "C");

}

//function to output current time to serial monitor
void printTime() {
  rtc.refresh();
  Serial1.println(String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second()));
}



//function to read serial input string start with "#" only
void sendAlerts(String str){
  if(str.startsWith("#")){
    sendMessage(str);
  }
}


void updateSerial(){
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}

//create function to send messages using gsm module
void sendMessage(String str){
  mySerial.println("AT+CMGS=\"+94703487817\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  mySerial.print(str); //text content
  updateSerial();
  mySerial.write(26);
  // updateSerial();
}


//create function to identify critical heart rate 
void checkBPM(int bpm){
  if(bpm > 100){
    //send alert
    sendMessage("Patient: Janith heart rate is too high");
  }
  else if(bpm < 60){
    //send alert
    sendMessage("Patient: Janith heart rate is too low");
  }
}

//create function to identify critical body temperature
void checkBodyTemp(float bodyTemp){
  if(bodyTemp > 37.5){
    //send alert
    sendMessage("Patient: Janith Body temperature is too high");
  }
  else if(bodyTemp < 35.5){
    //send alert
    sendMessage("Patient: Janith body temperature is too low");
  }
}

//control bed lifting function
void bedLiftingFunction(){
  if(digitalRead(bedUpPin)==HIGH && digitalRead(motorPinDown)==HIGH){
    invalid = true;
  }else{
    invalid = false;
  }
  if(digitalRead(bedUpPin)==HIGH && !invalid){
    digitalWrite(motorPinUp, HIGH);
  }
  if(digitalRead(motorPinDown)==HIGH && !invalid){
    digitalWrite(motorPinDown, HIGH);
  }
  if(digitalRead(bedUpPin)==LOW){
    digitalWrite(motorPinUp, LOW);
  }
  if(digitalRead(motorPinDown)==LOW){
    digitalWrite(motorPinDown, LOW);
  }
}

//use below function if above function is not working
//bed lifting function
void bedLiftingFunction2(){
  if((digitalRead(bedUpPin)==HIGH && digitalRead(bedDownPin==HIGH))||(digitalRead(motorPinDown)==HIGH && digitalRead(motorPinUp==HIGH))){
    invalid = true;
    digitalWrite(motorPinUp, LOW);
    digitalWrite(motorPinDown, LOW);
  }else{
    invalid = false;
  }
  if(digitalRead(bedUpPin)==HIGH && !invalid){
    digitalWrite(motorPinUp, HIGH);
  }
  if(digitalRead(bedDownPin)==HIGH && !invalid){
    digitalWrite(motorPinDown, HIGH);
  }

}



//to find patient is on the bed or not
int isWeightDetected() {
  const int serialPrintInterval = 1000;

  if (LoadCell.update())
    newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      if (i > 400.0) {
        functionStartTime = millis();
        return 1;
      }
      newDataReady = false;
      t = millis();
    }
  }
  return 0;
}

//to find over humidity or not
int overHumidity(){
  // delay(10);

  // sensors_event_t eventH;
  // dht.humidity().getEvent(&eventH);
  if (isnan(dht.readHumidity()) || isnan(dht.readTemperature())) {
    Serial.println(F("Error reading humidity!"));
    
  }
  else {
    if(dht.readHumidity()>80.0){
      return 1;
    }
  }
    return 0;
}

//to find over temperature or not
int overTemperature(){
  // delay(10);
  // dht.temperature().getEvent(&eventT);
  if (isnan(dht.readHumidity()) || isnan(dht.readTemperature())) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    if(dht.readTemperature()>30.0){
      return 1;
    }
  }
    return 0;
}



//if patient not in bed for 5 min send alert
void checkPatientThere(){
  if(millis() - functionStartTime > 300000){
    //send alert
    sendMessage("Patient: Janith is not on the bed");
  }
}


 

//vr recog function
void vrRecog(){
  int ret;
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    switch(buf[1]){
      case onRecord:
      case onRecord1:
      case onRecord2:


        /** turn on LED */
        // digitalWrite(led, HIGH);
        break;
      case offRecord:
      case offRecord1:
      case offRecord2:
        /** turn off LED*/
        // digitalWrite(led, LOW);
        break;
      default:
        Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
    // printVR(buf);
  }
}