int data = 85;
#include <Servo.h>
#include <Wire.h>
float time;
Servo servo;
void setup() {
  Serial.begin(115200); // set the baud rate
  Serial.println("Ready"); // print "Ready" once
  servo.attach(3);
}
void loop() {
  if (Serial.available()){
    data = Serial.read();
} 

servo.write(data);
delay(15);
}
