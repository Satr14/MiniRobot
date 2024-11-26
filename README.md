Tugas Pemrogaman Sistem Mekatronik

 ======================================
 ===           Mini Robot           ===
 ======================================
 
 ================================================================
 ===               Arduino Nano Wall Follower                 ===
 ================================================================

Not Use : RST,D8,D9,REF

Button
	RST : reset

Switch Komunikasi Antar Arduino
	1: RX -> TX
	2: TX -> RX

Modul wifi ESP diganti dengan modul Bluetooth
	D10 (RX/PWM) : RX
	D11 (TX/Mosi1) : TX
	3V3	: Vcc dan Ch_Pd/En
	GND	: GND dan GPIO 0
	Not Use : (reset),(GPIO 2)

IMU 6050
	3V3 : pin 1 (3v3)
	A4 : pin 4 (SDA)
	A5 : pin 3 (SCL)
	no use : sisanya 
Yang digunakan untuk heading hanya sumbu z saja

Ultrasonic Kanan 
	Trig : D7
	Echo : D6
	
Ultrasonic Kiri
	Trig : D3
	Echo : D2

Ultrasonic Depan
	Trig : D5
	Echo : D4

 ================================================================
 ===               Arduino Nano Line Follower                 ===
 ================================================================
Not Use : RST,REF,A7

LCD I2C
	Address: 0X27
	A4 : SDA
	A5 : SCL

Driver Motor (HW-166)
	PWMA : D3
	AIN2 : D2
	AIN1 : D4
	STBY : D9
	BIN1 : D7
	BIN2 : D8
	PWMB : D5

Buzzer 
	Transistor : D6

Button
	RST : reset
	D12(1) : D12
	D11(2) : D11
	D13(3) : D13

Led
	- : D10

Led & Infrared
	infrared 1 : A0
	infrared 2 : A1
	infrared 3 : A2
	infrared 4 : A3
	infrared 5 : A6
	
Pembacaan Putih <720
Pembacaan Hitam >880
