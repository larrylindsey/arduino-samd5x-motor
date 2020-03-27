/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

// setup input pot for velocity/position
int pot_center;
#define SPEED_MAX 255

#include <QuadratureEncoder.h>
// Max number of Encoders object you can create is 4. This example only uses 2.

Encoders encoder(2,3);  // Create an Encoder object named encoder, using digitalpin 2 & 3
long encoderCount_prev;


void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  pot_center = analogRead(0);
  encoderCount_prev = encoder.getEncoderCount();

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
}


void loop() {
  int pot = analogRead(0);
  int speed = constrain(pot - pot_center, -SPEED_MAX, SPEED_MAX);

  if (speed > 0) {
    myMotor->run(FORWARD);
    myMotor->setSpeed(abs(speed));  
  } else if (speed < 0) {
    myMotor->run(BACKWARD);
    myMotor->setSpeed(abs(speed));  
  } else {
    myMotor->run(RELEASE);
  }

  long encoderCount = encoder.getEncoderCount();
  int velocity = encoderCount - encoderCount_prev;
  encoderCount_prev = encoderCount;

  Serial.print("pot=");
  Serial.print(pot);
  Serial.print(" speed=");
  Serial.print(speed);
  
  Serial.print(" pos=");
  Serial.print(encoderCount);
  Serial.print(" vel=");
  Serial.println(velocity);

  delay(200);
}
