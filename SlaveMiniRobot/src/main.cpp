#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define pin motor
#define PWM1 3   // Motor A PWM
#define AIN1 2   // Motor A IN1
#define AIN2 4   // Motor A IN2
#define STBY 9   // Standby pin
#define PWM2 5   // Motor B PWM
#define BIN1 7   // Motor B IN1
#define BIN2 8   // Motor B IN2

// Define pin infrared sensors
const int IFR1 = A0;
const int IFR2 = A1;
const int IFR3 = A2;
const int IFR4 = A3;
const int IFR5 = A6;

const int led = 10;  // Pin LED for indication (optional)

// Buffer size for moving average
const int bufferSize = 10;

// Buffers for each sensor to store recent readings
int bufferIFR1[bufferSize] = {0};
int bufferIFR2[bufferSize] = {0};
int bufferIFR3[bufferSize] = {0};
int bufferIFR4[bufferSize] = {0};
int bufferIFR5[bufferSize] = {0};

// Index to keep track of the current position in the buffer
int bufferIndex = 0;

// Function to compute the moving average for a sensor
int movingAverage(int buffer[], int newValue) {
  buffer[bufferIndex] = newValue;  // Update the buffer with the new value
  int sum = 0;
  
  // Calculate the sum of all values in the buffer
  for (int i = 0; i < bufferSize; i++) {
    sum += buffer[i];
  }
  
  return sum / bufferSize;  // Return the average
}

// Set up motor
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

void motorB_CCW(int speed) { // Maju
  digitalWrite(AIN1, LOW);  // AIN1 LOW
  digitalWrite(AIN2, HIGH); // AIN2 HIGH
  analogWrite(PWM1, speed); // Set motor speed using PWM
}

void motorA_CW(int speed) {
  digitalWrite(BIN1, HIGH); // BIN1 HIGH
  digitalWrite(BIN2, LOW);  // BIN2 LOW
  analogWrite(PWM2, speed); // Set motor speed using PWM
}

void motorA_CCW(int speed) { // Maju
  digitalWrite(BIN1, LOW);  // BIN1 LOW
  digitalWrite(BIN2, HIGH); // BIN2 HIGH
  analogWrite(PWM2, speed); // Set motor speed using PWM
}

char Incoming_value = 0;
bool lineTracerMode = false;

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  setupMotors();
  Serial.begin(9600);
  lcd.init(); // Initialize the LCD
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Mini Robot");
  lcd.setCursor(2, 1);
  lcd.print("Mode: Manual");

  // Initialize infrared sensors and LED
  pinMode(IFR1, INPUT);
  pinMode(IFR2, INPUT);
  pinMode(IFR3, INPUT);
  pinMode(IFR4, INPUT);
  pinMode(IFR5, INPUT);
  pinMode(led, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    Incoming_value = Serial.read();
    Serial.println(Incoming_value);

    if (Incoming_value == 'A') {
      motorB_CW(150);
      motorA_CCW(150);
    } else if (Incoming_value == 'B') {
      motorB_CCW(150);
      motorA_CCW(150);
    } else if (Incoming_value == 'C') {
      motorB_CCW(150);
      motorA_CW(150);
    } else if (Incoming_value == 'D') {
      motorB_CCW(0);
      motorA_CCW(175);
    } else if (Incoming_value == 'F') {
      motorB_CCW(175);
      motorA_CCW(0);
    } else if (Incoming_value == 'G') {
      motorB_CW(150);
      motorA_CCW(150);
    } else if (Incoming_value == 'H') {
      motorB_CW(150);
      motorA_CW(150);
    } else if (Incoming_value == 'I') {
      motorB_CCW(150);
      motorA_CW(150);
    } else if (Incoming_value == 'S') {
      motorB_CW(0);
      motorA_CW(0);
    } else if (Incoming_value == 'L') {
      lineTracerMode = true;
      lcd.clear();
      lcd.print("Mode:");
      lcd.setCursor(0, 1);
      lcd.print("Line Follower");
    } else if (Incoming_value == 'M') {
      lineTracerMode = false;
      lcd.clear();
      lcd.setCursor(3, 0);
      lcd.print("Mini Robot");
      lcd.setCursor(2, 1);
      lcd.print("Mode: Manual");
    } else if (Incoming_value == 'N') {
      lineTracerMode = false;
      lcd.clear();
      lcd.print("Mode:");
      lcd.setCursor(0, 1);
      lcd.print("Obstacle Avoider");
    } else if (Incoming_value == 'W') {
      lineTracerMode = false;
      lcd.clear();
      lcd.print("Mode:");
      lcd.setCursor(0, 1);
      lcd.print("Wall Follower");
    }
  }

  int rawIFR1 = analogRead(IFR1);
  int rawIFR2 = analogRead(IFR2);
  int rawIFR3 = analogRead(IFR3);
  int rawIFR4 = analogRead(IFR4);
  int rawIFR5 = analogRead(IFR5);
    
  // Apply moving average to each sensor reading
  int avgIFR1 = movingAverage(bufferIFR1, rawIFR1);
  int avgIFR2 = movingAverage(bufferIFR2, rawIFR2);
  int avgIFR3 = movingAverage(bufferIFR3, rawIFR3);
  int avgIFR4 = movingAverage(bufferIFR4, rawIFR4);
  int avgIFR5 = movingAverage(bufferIFR5, rawIFR5);
    
  // Increment buffer index and reset when it reaches the buffer size
  bufferIndex = (bufferIndex + 1) % bufferSize;
  // Tampilkan hasil pembacaan sensor di Serial Monitor

  String dataPrefix = "X";
  String dataLine = dataPrefix + String(avgIFR1) + "?" + String(avgIFR2) + "?" + String(avgIFR3) + "?" + String(avgIFR4) + "?" + String(avgIFR5);
  Serial.println(dataLine);
   
  // motor A kiri---- CCW maju
  // motor B kanan--- CCW maju
  if (lineTracerMode) {
    // Jika sensor 3 mendeteksi garis (tengah)
    if (avgIFR3 < 900) {
      // Maju
      motorA_CCW(150);
      motorB_CCW(150);
      digitalWrite(led, HIGH);
    }
    // Jika sensor 2 mendeteksi garis (sedikit kiri)
    else if (avgIFR2 < 900) {
      // Belok kiri sedikit
      motorA_CCW(150);
      motorB_CCW(125);
      digitalWrite(led, HIGH);
    }
    // Jika sensor 4 mendeteksi garis (sedikit kanan)
    else if (avgIFR4 < 900) {
      // Belok kanan sedikit
      motorA_CCW(125);
      motorB_CCW(150);
      digitalWrite(led, HIGH);
    }
    // Jika sensor 1 mendeteksi garis (kiri jauh)
    else if (avgIFR1 < 900) {
      // Belok kiri tajam
      motorA_CCW(150);
      motorB_CCW(100);
      digitalWrite(led, HIGH);
    }
    // Jika sensor 5 mendeteksi garis (kanan jauh)
    else if (avgIFR5 < 900) {
      // Belok kanan tajam
      motorA_CCW(100);
      motorB_CCW(150);
      digitalWrite(led, HIGH);
    }
    // Jika tidak ada sensor yang mendeteksi garis
    else {
      // Berhenti
      motorA_CW(0);
      motorB_CW(0);
      digitalWrite(led, LOW);
    }
  }
}

// #include <Arduino.h>

// // Define pin motor
// #define PWM1 3   // Motor A PWM
// #define AIN1 2   // Motor A IN1
// #define AIN2 4   // Motor A IN2
// #define STBY 9   // Standby pin
// #define PWM2 5   // Motor B PWM
// #define BIN1 7   // Motor B IN1
// #define BIN2 8   // Motor B IN2

// // Define pin infrared sensors
// const int IFR1 = A0;
// const int IFR2 = A1;
// const int IFR3 = A2;
// const int IFR4 = A3;
// const int IFR5 = A6;

// const int led = 10;  // Pin LED for indication (optional)

// // Buffer size for moving average
// const int bufferSize = 10;

// // Buffers for each sensor to store recent readings
// int bufferIFR1[bufferSize] = {0};
// int bufferIFR2[bufferSize] = {0};
// int bufferIFR3[bufferSize] = {0};
// int bufferIFR4[bufferSize] = {0};
// int bufferIFR5[bufferSize] = {0};

// // Index to keep track of the current position in the buffer
// int bufferIndex = 0;

// // Function to compute the moving average for a sensor
// int movingAverage(int buffer[], int newValue) {
//   buffer[bufferIndex] = newValue;  // Update the buffer with the new value
//   int sum = 0;
  
//   // Calculate the sum of all values in the buffer
//   for (int i = 0; i < bufferSize; i++) {
//     sum += buffer[i];
//   }
  
//   return sum / bufferSize;  // Return the average
// }

// // Set up motor
// void setupMotors() {
//   pinMode(PWM1, OUTPUT);  // Motor A PWM
//   pinMode(AIN1, OUTPUT);  // Motor A IN1
//   pinMode(AIN2, OUTPUT);  // Motor A IN2
//   pinMode(STBY, OUTPUT);  // Standby pin
//   pinMode(PWM2, OUTPUT);  // Motor B PWM
//   pinMode(BIN1, OUTPUT);  // Motor B IN1
//   pinMode(BIN2, OUTPUT);  // Motor B IN2
  
//   digitalWrite(STBY, HIGH); // Ensure motors are not in standby
// }

// void motorB_CW(int speed) {
//   digitalWrite(AIN1, HIGH); // AIN1 HIGH
//   digitalWrite(AIN2, LOW);  // AIN2 LOW
//   analogWrite(PWM1, speed); // Set motor speed using PWM
// }

// void motorB_CCW(int speed) { // Maju
//   digitalWrite(AIN1, LOW);  // AIN1 LOW
//   digitalWrite(AIN2, HIGH); // AIN2 HIGH
//   analogWrite(PWM1, speed); // Set motor speed using PWM
// }

// void motorA_CW(int speed) {
//   digitalWrite(BIN1, HIGH); // BIN1 HIGH
//   digitalWrite(BIN2, LOW);  // BIN2 LOW
//   analogWrite(PWM2, speed); // Set motor speed using PWM
// }

// void motorA_CCW(int speed) { // Maju
//   digitalWrite(BIN1, LOW);  // BIN1 LOW
//   digitalWrite(BIN2, HIGH); // BIN2 HIGH
//   analogWrite(PWM2, speed); // Set motor speed using PWM
// }

// char Incoming_value = 0;
// bool lineTracerMode = false;

// void setup() {
//   setupMotors();
//   Serial.begin(9600);

//   // Initialize infrared sensors and LED
//   pinMode(IFR1, INPUT);
//   pinMode(IFR2, INPUT);
//   pinMode(IFR3, INPUT);
//   pinMode(IFR4, INPUT);
//   pinMode(IFR5, INPUT);
//   pinMode(led, OUTPUT);
// }

// void loop() {
//   if (Serial.available() > 0) {
//     Incoming_value = Serial.read();
//     Serial.println(Incoming_value);

//     if (Incoming_value == 'A') {
//       motorB_CW(150);
//       motorA_CCW(150);
//     } else if (Incoming_value == 'B') {
//       motorB_CCW(150);
//       motorA_CCW(150);
//     } else if (Incoming_value == 'C') {
//       motorB_CCW(150);
//       motorA_CW(150);
//     } else if (Incoming_value == 'D') {
//       motorB_CCW(0);
//       motorA_CCW(175);
//     } else if (Incoming_value == 'F') {
//       motorB_CCW(175);
//       motorA_CCW(0);
//     } else if (Incoming_value == 'G') {
//       motorB_CW(150);
//       motorA_CCW(150);
//     } else if (Incoming_value == 'H') {
//       motorB_CW(150);
//       motorA_CW(150);
//     } else if (Incoming_value == 'I') {
//       motorB_CCW(150);
//       motorA_CW(150);
//     } else if (Incoming_value == 'S') {
//       motorB_CW(0);
//       motorA_CW(0);
//     } else if (Incoming_value == 'L') {
//       lineTracerMode = true; 
//     } else if (Incoming_value == 'M') {
//       lineTracerMode = false;
//     } else if (Incoming_value == 'N') {
//       lineTracerMode = false;
//     }
//   }

//     int rawIFR1 = analogRead(IFR1);
//     int rawIFR2 = analogRead(IFR2);
//     int rawIFR3 = analogRead(IFR3);
//     int rawIFR4 = analogRead(IFR4);
//     int rawIFR5 = analogRead(IFR5);
    
//     // Apply moving average to each sensor reading
//     int avgIFR1 = movingAverage(bufferIFR1, rawIFR1);
//     int avgIFR2 = movingAverage(bufferIFR2, rawIFR2);
//     int avgIFR3 = movingAverage(bufferIFR3, rawIFR3);
//     int avgIFR4 = movingAverage(bufferIFR4, rawIFR4);
//     int avgIFR5 = movingAverage(bufferIFR5, rawIFR5);
    
//     // Increment buffer index and reset when it reaches the buffer size
//     bufferIndex = (bufferIndex + 1) % bufferSize;
//     // Tampilkan hasil pembacaan sensor di Serial Monitor

// String dataPrefix = "X";
// String dataLine = dataPrefix + String(avgIFR1) + "?" + String(avgIFR2) + "?" + String(avgIFR3) + "?" + String(avgIFR4) + "?" + String(avgIFR5);
// Serial.println(dataLine);
   
// // motor A kiri---- CCW maju
// // motor B kanan--- CCW maju
//   if (lineTracerMode) {
//     // Jika sensor 3 mendeteksi garis (tengah)
//     if (avgIFR3 < 900) {
//       // Maju
//       motorA_CCW(150);
//       motorB_CCW(150);
//       digitalWrite(led, HIGH);
//     }
//     // Jika sensor 2 mendeteksi garis (sedikit kiri)
//     else if (avgIFR2 < 900) {
//       // Belok kiri sedikit
//       motorA_CCW(150);
//       motorB_CCW(125);
//       digitalWrite(led, HIGH);
//     }
//     // Jika sensor 4 mendeteksi garis (sedikit kanan)
//     else if (avgIFR4 < 900) {
//       // Belok kanan sedikit
//       motorA_CCW(125);
//       motorB_CCW(150);
//       digitalWrite(led, HIGH);
//     }
//     // Jika sensor 1 mendeteksi garis (kiri jauh)
//     else if (avgIFR1 < 900) {
//       // Belok kiri tajam
//       motorA_CCW(150);
//       motorB_CCW(100);
//       digitalWrite(led, HIGH);
//     }
//     // Jika sensor 5 mendeteksi garis (kanan jauh)
//     else if (avgIFR5 < 900) {
//       // Belok kanan tajam
//       motorA_CCW(100);
//       motorB_CCW(150);
//       digitalWrite(led, HIGH);
//     }
//     // Jika tidak ada sensor yang mendeteksi garis
//     else {
//       // Berhenti
//       motorA_CW(0);
//       motorB_CW(0);
//       digitalWrite(led, LOW);
//     }
//   }
// }
  








