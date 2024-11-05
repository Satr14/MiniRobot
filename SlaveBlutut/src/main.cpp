#include <Arduino.h>

//define pin motor
#define PWM1 3   // Motor A PWM
#define AIN1 2   // Motor A IN1
#define AIN2 4   // Motor A IN2
#define STBY 9   // Standby pin
#define PWM2 5   // Motor B PWM
#define BIN1 7   // Motor B IN1
#define BIN2 8   // Motor B IN2

//set up motor
void setupMotors() {
  pinMode(PWM1, OUTPUT);  // Motor A PWM
  pinMode(AIN1, OUTPUT);  // Motor A IN1
  pinMode(AIN2, OUTPUT);  // Motor A IN2
  pinMode(STBY, OUTPUT);  // Standby pin
  pinMode(PWM2, OUTPUT);  // Motor B PWM
  pinMode(BIN1, OUTPUT);  // Motor B IN1
  pinMode(BIN2, OUTPUT);  // Motor B IN2
  
  digitalWrite(STBY, HIGH); // Ensure motors are not in standby
}

void motorB_CW(int speed) {
  digitalWrite(AIN1, HIGH); // AIN1 HIGH
  digitalWrite(AIN2, LOW);  // AIN2 LOW
  analogWrite(PWM1, speed); // Set motor speed using PWM
}

// Function to rotate Motor A in counterclockwise (CCW) direction
void motorB_CCW(int speed) { //maju
  digitalWrite(AIN1, LOW);  // AIN1 LOW
  digitalWrite(AIN2, HIGH); // AIN2 HIGH
  analogWrite(PWM1, speed); // Set motor speed using PWM
}

// Function to rotate Motor B in clockwise (CW) direction
void motorA_CW(int speed) {
  digitalWrite(BIN1, HIGH); // BIN1 HIGH
  digitalWrite(BIN2, LOW);  // BIN2 LOW
  analogWrite(PWM2, speed); // Set motor speed using PWM
}

// Function to rotate Motor B in counterclockwise (CCW) direction
void motorA_CCW(int speed) { //Maju
  digitalWrite(BIN1, LOW);  // BIN1 LOW
  digitalWrite(BIN2, HIGH); // BIN2 HIGH
  analogWrite(PWM2, speed); // Set motor speed using PWM
}

char Incoming_value = 0;

int q = 0;

void setup() {
  setupMotors();  
  Serial.begin(9600);
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= 100) {
    lastUpdateTime = currentTime;    

    q = (q + 1) % 1001;

    // if (sendData) {
    Serial.print(q);
    Serial.print("-");
  }

  if(Serial.available() > 0) {
    Incoming_value = Serial.read();      
    Serial.println(Incoming_value); 

    // Kiri Maju
    if(Incoming_value == 'A') {
      motorB_CW(255);
      motorA_CCW(255);
    // Maju  
    } else if(Incoming_value == 'B') {
      motorB_CCW(255);
      motorA_CCW(255);
    // Kanan Maju
    } else if(Incoming_value == 'C') {
      motorB_CCW(255);
      motorA_CW(255);
    // Kiri
    } else if(Incoming_value == 'D') {
      motorB_CCW(0);
      motorA_CCW(175);
    // Kanan 
    } else if(Incoming_value == 'F') {
      motorB_CCW(175);
      motorA_CCW(0);
    // Kiri Mundur
    } else if(Incoming_value == 'G') {
      motorB_CW(255);
      motorA_CCW(255);
    // Mundur    
    } else if(Incoming_value == 'H') { 
      motorB_CW(255);
      motorA_CW(255);      
    // Kanan Mundur
    } else if(Incoming_value == 'I') {
      motorB_CCW(255);
      motorA_CW(255);
    // Stop
    } else if(Incoming_value == 'S') {
      motorB_CW(0);
      motorA_CW(0);
    }
  }
}
