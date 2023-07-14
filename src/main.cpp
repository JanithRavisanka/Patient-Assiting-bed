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


//function to read temperature
void readBodyTemp();

//fuction to read pulse
void readPulse();

//function to read humidity and room temperature
float readDHT();

void createDisplay();

void printTime();

void sendAlerts(String str);



void setup() {
  //serial connection with NodeMCU
  Serial.begin(9600);
  Serial1.begin(9600);

  //body temp sensor
  bodyTempSensor.begin();

  //pulse sensor
  // pulseSensor.analogInput(PulseWire);   
  // pulseSensor.blinkOnPulse(LED);       //auto-magically blink Arduino's LED with heartbeat.
  // pulseSensor.setThreshold(Threshold);  
  // if (pulseSensor.begin()) {
  //   Serial.println("We created a pulseSensor Object !");
  // }

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
}


void loop() { 
  
   // Calls function on our pulseSensor object that returns BPM as an "int".
  // "myBPM" hold this BPM value now.
  // if (pulseSensor.sawStartOfBeat()) {  
  //   int myBPM = pulseSensor.getBeatsPerMinute()           // Constantly test to see if "a beat happened".   
  //   Serial.println("bpm = " + String(myBPM));            
  // }
  
  // readBodyTemp();
  // readPulse();
  // readDHT();
  createDisplay();
  // printTime();


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
//function to split string "=" and make it into a key value pair



//function to read serial input string start with "#" only
void sendAlerts(String str){
  if(str.startsWith("#")){
    //send alerts using gsm module
  }
}



