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

//DHT sensor
#define DHTPIN 8
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//RTC and LCD
uRTCLib rtc(0x68);
LiquidCrystal_I2C lcd(0x27, 16, 2);


//function to read temperature
void readBodyTemp();

//fuction to read pulse
void readPulse();

//function to read humidity and room temperature
void readDHT();

void createDisplay();


void setup() {
  //serial connection with NodeMCU
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
}


void loop() { 
  
   // Calls function on our pulseSensor object that returns BPM as an "int".
  // "myBPM" hold this BPM value now.
  // if (pulseSensor.sawStartOfBeat()) {  
  //   int myBPM = pulseSensor.getBeatsPerMinute()           // Constantly test to see if "a beat happened".   
  //   Serial.println("bpm = " + String(myBPM));            
  // }
  
  readBodyTemp();
  readPulse();
  readDHT();
  createDisplay();
  printTime();

}



//read body temperature
void readBodyTemp() {
  bodyTempSensor.requestTemperatures();
  Serial1.println("bTemp = " + String(bodyTempSensor.getTempCByIndex(0)));
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
    Serial1.println("bpm = " + String(bpm));
  }
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial1.println("humidity = " + String(h));
  Serial1.println("bodytemp = " + String(t));
}


void createDisplay(){
  rtc.refresh();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM: " + String(random(60, 100)));
  lcd.setCursor(0, 1);
  lcd.print("Body Temp: " + String(dht.readHumidity()) + "C");

}

//function to output current time to serial monitor
void printTime() {
  rtc.refresh();
  Serial1.println(String(rtc.hour()) + ":" + String(rtc.minute()) + ":" + String(rtc.second()));
}

