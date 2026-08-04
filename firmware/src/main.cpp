#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Default address 0x40
void setup() {
    Serial.begin(9600);           // Init serial for debugging
    pwm.begin();                  // Init PCA9685
    pwm.setPWMFreq(60);           // 60 Hz for servos
}
void loop() {
    // Servo 0
    int potValue0 = analogRead(A0);           // Read A0
    int servoPos0 = map(potValue0, 0, 1023, 125, 575); // Map to pulse range
    pwm.setPWM(0, 0, servoPos0);              // Set servo 0
    // Servo 1
    int potValue1 = analogRead(A1);           // Read A1
    int servoPos1 = map(potValue1, 0, 1023, 125, 575); // Map to pulse range
    pwm.setPWM(1, 0, servoPos1);              // Set servo 1
    // Servo 2
    int potValue2 = analogRead(A2);           // Read A2
    int servoPos2 = map(potValue2, 0, 1023, 125, 575); // Map to pulse range
    pwm.setPWM(2, 0, servoPos2);              // Set servo 2
    // Servo 3
    int potValue3 = analogRead(A3);           // Read A3
    int servoPos3 = map(potValue3, 0, 1023, 125, 575); // Map to pulse range
    pwm.setPWM(6, 0, servoPos3);              // Set servo 3
    // Servo 4
    int potValue4 = analogRead(A4);           // Read A4
    int servoPos4 = map(potValue4, 0, 1023, 125, 575); // Map to pulse range
    pwm.setPWM(4, 0, servoPos4);              // Set servo 4
    // Servo 5
    int potValue5 = analogRead(A5);           // Read A5
    int servoPos5 = map(potValue5, 0, 1023, 125, 575); // Map to pulse range
    pwm.setPWM(5, 0, servoPos5);              // Set servo 5
    delay(20);   // Small delay for smooth updates
}