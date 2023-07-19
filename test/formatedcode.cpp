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

//display
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif


//Liquid Crystal display
uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long refreshTime = 0;


//voice recognition
VR myVR(10, 11);   
uint8_t records[7]; // save record
uint8_t buf[64];

#define onRecord    (0)
#define offRecord   (1) 
#define onRecord1    (2)
#define offRecord1   (3) 
#define onRecord2    (4)
#define offRecord2   (5) 


//Load cell
const int HX711_dout = 25;  // MCU > HX711 dout pin
const int HX711_sck = 23;  // MCU > HX711 sck pin
const int RELAYPIN = 7;    // MCU > LED pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
unsigned long t = 0;
bool newDataReady = 0;
unsigned long functionStartTime = 0;


//touch sensors
const int bedDownPin = 12;
const int bedStopPin = 53;
const int bedUpPin = 47;
unsigned long touchInterval = 0;

//motor
const int motorPinUp = 27; //up
const int motorPinDown = 29; //down

unsigned long functionStartTimeT = 0;
unsigned long functionEndTime = 0;


//pulse sensor
const int pulseWire = A2;
int Threshold = 130;
int count = 0;
int bpm = 0;
unsigned long lastPulse = 0;
int signal = 0;
unsigned long lastBPM = 0;
int currentBPM = 0;
unsigned long lastCheck = 0;
int bpmArr[3] = {0,0,0};


//body temperature sensor
#define bodyTempPin 9
OneWire bTempPin(bodyTempPin);
DallasTemperature bodyTempSensor(&bTempPin);
int bodyTempArr[3] = {0,0,0};


//DHT sensor
#define DHTPIN 8
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);


//RTC
uRTCLib rtc;


//GSM module
SoftwareSerial gsmSerial(2, 3); // RX, TX


void readBodyTemp();
void readPulse();
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
void createDisplay();

void setup(){
    //serial connection with NodeMCU
    Serial.begin(115200);
    Serial1.begin(115200);

    //Display
    lcd.init();
    lcd.backlight();
    lcd.createChar(0, heart);
    lcd.clear();
    lcd.home();
    lcd.print("Initializing...");
    lcd.setCursor(0, 1);
    lcd.print("Please wait...");
    


    //Voice recognition module
    myVR.begin(9600);
    if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
    }else{
        Serial.println("Not find VoiceRecognitionModule.");
        Serial.println("Please check connection and restart Arduino.");
        while(1);
    }
    if(myVR.load((uint8_t)onRecord) >= 0){
        Serial.println("onRecord loaded");
    }
    if(myVR.load((uint8_t)offRecord) >= 0){
        Serial.println("offRecord loaded");
    }

    //Motor
    pinMode(motorPinUp, OUTPUT);
    pinMode(motorPinDown, OUTPUT);

    //Touch sensors
    pinMode(bedDownPin, INPUT);
    pinMode(bedStopPin, INPUT);
    pinMode(bedUpPin, INPUT);


    //Body temperature sensor
    bodyTempSensor.begin();

    //DHT sensor
    dht.begin();
    if (isnan(dht.readHumidity())) {
        Serial.println(F("Failed to read from DHT sensor!"));
        while (1){
            Serial1.println("Connecting to DHT sensor...");
        }   
    }

    //RTC
    URTCLIB_WIRE.begin();


    //GSM module
    gsmSerial.begin(9600);
    delay(1000);
    gsmSerial.println("AT"); //Once the handshake test is successful, it will back to OK
    updateSerial();
    gsmSerial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    gsmSerial.println("AT+CMGS=\"+94703487817\"");
    updateSerial();
    gsmSerial.print("Message send from Patient Assisting Bed.This is a testing message"); //test message to send
    updateSerial();
    gsmSerial.write(26);

    //Load cell
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
        Serial.println("LoaCell Startup is complete");
    }
    delay(1000);
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Assisting Bed"); 
    lcd.setCursor(0, 1);
    lcd.print("Patient Assisting Bed"); 
    lcd.setCursor(0, 0);
    delay(2000); 
    lcd.clear();
}

void loop(){
    readPulse();
    createDisplay();
    bedControl();
    sendSensorDataToNode();
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

void bedControl(){
    if(digitalRead(bedUpPin) == HIGH && (millis() - touchInterval > 1000)){
        touchInterval = millis();
        digitalWrite(motorPinDown, LOW);
        digitalWrite(motorPinUp, HIGH);
        Serial.println("Bed is going up");  
    }else if(digitalRead(bedDownPin) == HIGH && (millis() - touchInterval > 1000)){
        touchInterval = millis();
        digitalWrite(motorPinUp, LOW);
        digitalWrite(motorPinDown, HIGH);
        Serial.println("Bed is going down");
    }else if(digitalRead(bedStopPin) == HIGH && (millis() - touchInterval > 1000)){
        touchInterval = millis();
        digitalWrite(motorPinUp, LOW);
        digitalWrite(motorPinDown, LOW);
        Serial.println("Bed is stopped");
    }
}

void sendSensorDataToNode(){
    if(bpm>0){
        bodyTempSensor.requestTemperatures();
        float humidity = dht.readHumidity();
        float roomTemp = dht.readTemperature();
        float bodyTemp = bodyTempSensor.getTempCByIndex(0);
        currentBPM = bpm;  //show curent bpm in display

        Serial.println("#humidity =" + String(humidity) + "$");
        Serial.println("#roomTemp=" + String(roomTemp) + "$");
        Serial.println("#bodyTemp=" + String(bodyTemp) + "$");
        Serial.println("#bpm=" + String(bpm) + "$");
    } 
}

void readPulse() {
  signal = analogRead(pulseWire);
  if(signal > Threshold && millis() - lastPulse> 400){
    count++;
    lastPulse = millis(); //get the last time a pulse was read
  }
  if(millis() - lastBPM > 60000){
    bpm = count;
    sortBPM(bpm);
    count = 0; //reset count
    lastBPM = millis(); //get the last time a BPM was read
  }
}

void updateSerial(){
  delay(500);
  while (Serial.available()) {
    gsmSerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(gsmSerial.available()) {
    Serial.write(gsmSerial.read());//Forward what Software Serial received to Serial Port
  }
}

void sendMessage(String str){
  gsmSerial.println("AT+CMGS=\"+94703487817\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  gsmSerial.print(str); //text content
  updateSerial();
  gsmSerial.write(26);
  updateSerial();
  Serial.print(str);
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

//if patient not in bed for 5 min send alert
void checkPatientThere(){
  if(millis() - functionStartTimeT > 300000){
    //send alert
    sendMessage("Patient: Janith is not on the bed");
    functionStartTimeT = millis();
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
        digitalWrite(motorPinDown, LOW);
        digitalWrite(motorPinUp, HIGH);
        break;
      case offRecord:
      case offRecord1:
      case offRecord2:
        digitalWrite(motorPinUp, LOW);
        digitalWrite(motorPinDown, HIGH);
        break;
      default:
        Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
    // printVR(buf);
  }
}


//create queue sorting algo to store data in bpmArr
void sortBPM(int bpm){
  for(int i=0; i<3; i++){
    if(bpmArr[i]==0){
      bpmArr[i]=bpm;
      break;
    }
  }
  for(int i=0; i<3; i++){
    for(int j=i+1; j<3; j++){
      if(bpmArr[i]>bpmArr[j]){
        int temp = bpmArr[i];
        bpmArr[i] = bpmArr[j];
        bpmArr[j] = temp;
      }
    }
  }
}

//function to calculate average bpm
int averageBPM(){
  //check how much values are 0
  int count = 0;
  for(int i=0; i<3; i++){
    if(bpmArr[i]==0){
      count++;
    }
  }

  int sum = 0;
  for(int i=0; i<3; i++){
    sum += bpmArr[i];
  }
  if(count==3){
    return 0;
  }else{
    return sum/(3-count);
  }
  
}

//create queue sorting algo to store data in bodyTempArr
void sortBodyTemp(float bodyTemp){
  for(int i=0; i<3; i++){
    if(bodyTempArr[i]==0){
      bodyTempArr[i]=bodyTemp;
      break;
    }
  }
  for(int i=0; i<3; i++){
    for(int j=i+1; j<3; j++){
      if(bodyTempArr[i]>bodyTempArr[j]){
        int temp = bodyTempArr[i];
        bodyTempArr[i] = bodyTempArr[j];
        bodyTempArr[j] = temp;
      }
    }
  }
}

//function to calculate average body temp
int averageBodyTemp(){
  //check how much values are 0
  int count = 0;
  for(int i=0; i<3; i++){
    if(bodyTempArr[i]==0){
      count++;
    }
  }
  int sum = 0;
  for(int i=0; i<3; i++){
    sum += bodyTempArr[i];
  }
  if(count==3){
    return 0;
  }else{
    return sum/(3-count);
  }
}

//to find over humidity or not
int overHumidity(){
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

void createDisplay(){
    if(millis() - refreshTime > 1000){
          // bodyTempSensor.requestTemperatures();
        rtc.refresh();
        lcd.clear();
        lcd.setCursor(0,0);
        if(rtc.month() < 10){
            lcd.print("0" + String(rtc.month()));
        }else{
            lcd.print(rtc.month());
        }
        lcd.print("/");
        if(rtc.day() < 10){
            lcd.print("0" + String(rtc.day()));
        }else{
            lcd.print(rtc.day());
        }

        lcd.setCursor(7,0);
        if(rtc.hour()<12){
            lcd.print("0" + String(rtc.hour()));
        }  else{
            lcd.print(String(rtc.hour()));
        }
        lcd.print(":");
        lcd.setCursor(10,0);
        if(rtc.minute()<10){
            lcd.print("0"  + String(rtc.minute()));
        }  else{
            lcd.print(String(rtc.minute()));
        }
        lcd.setCursor(0, 1);
        lcd.print("bpm:"+String(bpm));
        lcd.setCursor(8, 1);
        lcd.print("T:"+String(bodyTempSensor.getTempCByIndex(0)));
    }

}
