#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Ultrasonic.h>

SoftwareSerial mySerial(11, 10); // TX, RX

char Incoming_value = 0;
bool Avoider = false;
bool Wall = false;

const int trigPinF = 5;
const int echoPinF = 4;
const int trigPinR = 7;
const int echoPinR = 6;
const int trigPinL = 3;
const int echoPinL = 2;



Ultrasonic ultrasonic1(trigPinF, echoPinF); // Front
Ultrasonic ultrasonic2(trigPinR, echoPinR); // Right
Ultrasonic ultrasonic3(trigPinL, echoPinL); // Left

int irValues[5] = {0, 0, 0, 0, 0};

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
}


void UltrasonicSensor(){
  int distanceL = ultrasonic3.read(CM); // Left
  int distanceF = ultrasonic1.read(CM); // Front
  int distanceR = ultrasonic2.read(CM); // Right

  // Print the distance values
  mySerial.print(distanceL);
  mySerial.print("|");
  mySerial.print(distanceF);
  mySerial.print("|");
  mySerial.print(distanceR);
  mySerial.print("|");

    if (Avoider) {
      if (distanceF < 15) {
        if (distanceL < 15 && distanceR < 15) {
          Serial.println('H'); // Mundur
        } else if (distanceL < 15) {
          Serial.println('C'); // Kanan Depan
        } else if (distanceR < 15) {
          Serial.println('A'); // Kiri Depan
        } else {
          Serial.println('C'); // Kanan Depan (default action if front is blocked)
        }
      } else if (distanceL < 15) {
        Serial.println('C'); // Kanan Depan if left is blocked
      } else if (distanceR < 15) {
        Serial.println('A'); // Kiri Depan if right is blocked
      } else {
        Serial.println('B'); // Maju
      }
    }

  
if (Wall) {
  const int MAX_DIS = 30;
  const int FOLLOW_DISTANCE = 20;     // Ideal following distance
  const int TURN_THRESHOLD = 10;      // Threshold for turning
 
  if (distanceL < FOLLOW_DISTANCE && distanceF > MAX_DIS) {
        Serial.println('A');
      if (distanceF <= FOLLOW_DISTANCE && distanceF > TURN_THRESHOLD) {
        Serial.println('B');
        }
    }
  if (distanceR < FOLLOW_DISTANCE && distanceF > MAX_DIS) {
        Serial.println('C');
      if (distanceF <= FOLLOW_DISTANCE && distanceF > TURN_THRESHOLD) {
        Serial.println('B');
        }
      }
  if (distanceF <= FOLLOW_DISTANCE && distanceF > TURN_THRESHOLD && distanceL > MAX_DIS && distanceR > MAX_DIS) {
      Serial.println('B');
    }else if(distanceF > MAX_DIS && distanceL > MAX_DIS && distanceR > MAX_DIS) {
      Serial.println('S'); 
    } else if (distanceF < TURN_THRESHOLD) {
      Serial.println('H');
    }
}
}

void parseIRData(String data) {
  // Periksa awalan data
  if (data.startsWith("X")) {
    int startIndex = 1; // Abaikan awalan "X"
    int valueIndex = 0;
    for (int i = startIndex; i < data.length(); i++) {
      if (data.charAt(i) == '?' || i == data.length() - 1) {
        String valueStr = data.substring(startIndex, i);
        int value = valueStr.toInt();
        
        // Abaikan nilai di luar range 0-999
        if (value >= 0 && value <= 999) {
          irValues[valueIndex] = value;
        }
        
        startIndex = i + 1;
        valueIndex++;
        
        if (valueIndex >= 5) break; // Kita hanya ingin 5 nilai
      }
    }
  }

    mySerial.print(irValues[0]);
    mySerial.print("|");
    mySerial.print(irValues[1]);
    mySerial.print("|");
    mySerial.print(irValues[2]);
    mySerial.print("|");
    mySerial.print(irValues[3]);
    mySerial.print("|");
    mySerial.print(irValues[4]);
    mySerial.println();
  } 

void Infraredsensor(){
 if (Serial.available() > 0) {
    String irData = Serial.readStringUntil('\n');
    parseIRData(irData);
  }
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= 200) {
    lastUpdateTime = currentTime;

    UltrasonicSensor();
    Infraredsensor();
  }

  if (mySerial.available() > 0) {
    Incoming_value = mySerial.read();
    Serial.println(Incoming_value);

    if (Incoming_value == 'N') {
      Avoider = true; // Switch to auto mode2
      Wall = false;
    } else if (Incoming_value == 'M') {
      Avoider = false;
      Wall = false;
      Serial.println('S');  // Switch to manual mode
    } else if (Incoming_value == 'L') {
      Avoider = false;
      Wall = false;
      Serial.println('S');
    } else if (Incoming_value == 'W') {
      Wall = true;
      Avoider = false;
      Serial.println('S');
    }
  }
}

