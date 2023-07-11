#define USE_ARDUINO_INTERRUPTS true
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PulseSensorPlayground.h>
#include "DHT.h"




//body temperature sensor
#define bodyTempPin 9
OneWire bTempPin(bodyTempPin);
DallasTemperature bodyTempSensor(&bTempPin);

//heart pulse sensor
PulseSensorPlayground pulseSensor;
const int PulseWire = A2;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED = LED_BUILTIN;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;  

//DHT sensor
#define DHTPIN 8
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);


//function to read temperature
void readBodyTemp();

//fuction to read pulse
void readPulse();

//function to read humidity and room temperature
void readDHT();


void setup() {
  //serial connection with NodeMCU
  Serial1.begin(9600);

  //body temp sensor
  bodyTempSensor.begin();

  //pulse sensor
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);  
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");
  }

  //dht sensor
  dht.begin();

  if (isnan(dht.readHumidity())) {
    Serial.println(F("Failed to read from DHT sensor!"));
    //create infinite while loop
    while (1){
      Serial1.println("Connecting to DHT sensor...");
    }
  }
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
}



//read body temperature
void readBodyTemp() {
  bodyTempSensor.requestTemperatures();
  Serial1.println("bTemp = " + String(bodyTempSensor.getTempCByIndex(0)));
  delay(10);
}
//create function to output random hr values without sensor readings
void readPulse() {
  int pulse = random(60, 100);
  Serial1.println("pulse = " + String(pulse));
}

void readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial1.println("humidity = " + String(h));
  Serial1.println("bodytemp = " + String(t));
}

