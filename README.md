# IOT DC MOTOR 
int motorPin = 9;
void setup() {
 pinMode(motorPin, OUTPUT);
 Serial.begin(9600);
 while (! Serial);
 Serial.println("Speed 0 to 255");
}
void loop() {
 if (Serial.available()) {
 int speed = Serial.parseInt();
 if (speed >= 0 && speed <= 255) {
 analogWrite(motorPin, speed);
 }
 }
}
Temperature Sensor
Interfacing of Temperature Sensor (LM35):
#include<LiquidCrystal.h>
LiquidCrystal lcd(7,6,5,4,3,2);
void setup()
{
Serial.begin(19200);
lcd.begin(16,2);
}
void loop()
{
int analogvalue = analogRead(A3);
int pecent = map(analogvalue,0,1023,0,600);
Serial.print("analogvalue= ");
Serial.println(analogvalue);
lcd.setCursor(0,0);
lcd.print(analogvalue);
Serial.print("Pecent= ");
Serial.println(pecent);
lcd.setCursor(0,1);
lcd.print(pecent);
delay(1000);
lcd.clear ();
}

Interfacing of IR Sensor:
int ledPin=13;
int inputPin=2;
int val=0;
void setup()
{
pinMode(13,OUTPUT);
pinMode( inputPin, INPUT);
Serial.begin(9600);
}
void loop()
{
val=digitalRead(inputPin); // check the pin status (High=1/Low=0) //Active Low
output
if(val==HIGH)
{
Serial.print(“Object Absent\n");
digitalWrite(13,LOW);
}
else
{
Serial.print("Object Present\n");
digitalWrite(13,HIGH);
}
}

Interfacing of Ultrasonic Sensor:
#include <LiquidCrystal.h>
LiquidCrystal lcd (12,11,5,4,3,2);
// defining the pins
const int trigPin = 10;
const int echoPin = 9;
// defining variables
long duration;
int distance;
void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication
lcd.begin(16,2);
}
void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
lcd.setCursor (0,0);
lcd.print("Distance: ");
delay(1000);
lcd.setCursor (0,1);
lcd.print(distance);
lcd.print("cm");
}
