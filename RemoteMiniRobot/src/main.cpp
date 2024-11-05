#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>

SoftwareSerial mySerial(11, 10); // TX, RX

char Incoming_value = 0;
// static bool sendData = false;

const int trigPinF = 5;
const int echoPinF = 4;
const int trigPinR = 7;
const int echoPinR = 6;
const int trigPinL = 3;
const int echoPinL = 2;


Ultrasonic ultrasonic1(5, 4);	  // ngarep
Ultrasonic ultrasonic2(7, 6);		// tengen
Ultrasonic ultrasonic3(3, 2);		// kiwo

void setup() 
{
  mySerial.begin(9600);         
  Serial.begin(9600);  
}

void loop()
{
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= 100) {
    lastUpdateTime = currentTime;    
    // if (sendData) {
    mySerial.print(ultrasonic3.read(CM));
    mySerial.print("-"); 
    mySerial.print(ultrasonic1.read(CM));
    mySerial.print("-"); 
    mySerial.print(ultrasonic2.read(CM));
    mySerial.println("-");

  }

  if(mySerial.available() > 0)  
  {
    Incoming_value = mySerial.read();      
    Serial.println(Incoming_value); 
  }   

  
}

