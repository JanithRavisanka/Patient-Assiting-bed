#define USE_ARDUINO_INTERRUPTS true
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PulseSensorPlayground.h>



//body temperature sensor
#define bodyTempPin 9
OneWire bTempPin(bodyTempPin);
DallasTemperature bodyTempSensor(&bTempPin);

//heart pulse sensor

PulseSensorPlayground pulseSensor;
const int PulseWire = A2;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED = LED_BUILTIN;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;  


//function to read temperature
void readBodyTemp();

//fuction to read pulse
int readPulse();


void setup() {
  Serial1.begin(9600);
  bodyTempSensor.begin();

  //pulse sensor
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);  
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");
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
}


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


