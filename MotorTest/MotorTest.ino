/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <QuadratureEncoder.h>
#include <PID_v1.h>

#define SPEED_MAX 255


class PIDControl {
 public:

  PIDControl(int pin_a, int pin_b, int motor) : afms_(Adafruit_MotorShield()), encoder_(Encoders(pin_a, pin_b)) {
    afms_.begin();
    motor_ = afms_.getMotor(motor);    
    previous_count_ = encoder_.getEncoderCount();

    motor_->setSpeed(0);
    motor_->run(FORWARD);
    motor_->run(RELEASE);
  }

  void update() {
    motorVelocity_(target_velocity_);
  }

  void setVelocity(int velocity) {
    target_velocity_ = velocity;
  }

  void motorVelocity_(int velocity) {
    if (velocity == last_motor_velocity_) {
      return;
    }

    last_motor_velocity_ = velocity;

    if (velocity > 0) {
      motor_->run(FORWARD);
      motor_->setSpeed(abs(velocity));
    } else if (velocity < 0) {
      motor_->run(BACKWARD);
      motor_->setSpeed(abs(velocity));  
    } else {
      motor_->run(RELEASE);
    }
  }

 private: 
  Adafruit_MotorShield afms_;
  Encoders encoder_;
  
  Adafruit_DCMotor* motor_;
  long previous_count_;
  
  int target_velocity_;
  int last_motor_velocity_;
};

PIDControl* g_PIDControl;

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  
  g_PIDControl = new PIDControl(2, 3, 1);
}


void loop() {
  int v;
  if (millis() < 5000) {
    v = 255;
  } else {
    v = 0;
  }

  g_PIDControl->setVelocity(v);
  g_PIDControl->update();
  
//  long encoderCount = encoder.getEncoderCount();
//  int velocity = encoderCount - encoderCount_prev;
//  encoderCount_prev = encoderCount;

//  Serial.print("pot=");
//  Serial.print(pot);
//  Serial.print(" speed=");
//  Serial.print(speed);
//  
//  Serial.print(" pos=");
//  Serial.print(encoderCount);
//  Serial.print(" vel=");
//  Serial.println(velocity);

  delay(200);
}
