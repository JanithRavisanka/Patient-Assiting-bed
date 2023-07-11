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
#define USE_ARDUINO_INTERRUPTS true
PulseSensorPlayground pulseSensor;
const int PulseWire = A2;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED = LED_BUILTIN;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;  


//function to read temperature
void readBodyTemp();

//fuction to read pulse
void readPulse();


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
  readBodyTemp();
  int myBPM = pulseSensor.getBeatsPerMinute();  // Calls function on our pulseSensor object that returns BPM as an "int".
  // "myBPM" hold this BPM value now.
  if (pulseSensor.sawStartOfBeat()) {            // Constantly test to see if "a beat happened".
    myBPM = pulseSensor.getBeatsPerMinute();                 
  }
  Serial.println(myBPM); 
  delay(20);
}


void readBodyTemp() {
  bodyTempSensor.requestTemperatures();
  Serial1.println(bodyTempSensor.getTempCByIndex(0));
  delay(10);
}

void readPulse() {
                    // considered best practice in a simple sketch.
}