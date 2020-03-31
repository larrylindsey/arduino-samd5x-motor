/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
#include <QuadratureEncoder.h>
#include <Wire.h>

#define V_MAX_REV_PER_S 1.25
#define ENCODER_COUNT_PER_REV 9600

class PIDControl {
 public:
  PIDControl(int pin_a, int pin_b, int motor, int count_per_revolution,
             float kp, float ki, float kd)
      : afms_(Adafruit_MotorShield()),
        encoder_(Encoders(pin_a, pin_b)),
        count_per_revolution_(count_per_revolution),
        stop_(false) {
    afms_.begin();
    motor_ = afms_.getMotor(motor);
    motor_->setSpeed(0);
    motor_->run(FORWARD);
    motor_->run(RELEASE);

    pid_ =
        new PID(&pid_input_, &pid_output_, &pid_setpoint_, kp, ki, kd, DIRECT);
    pid_->SetMode(AUTOMATIC);
    pid_->SetOutputLimits(-1024, 1024);

    previous_count_ = encoder_.getEncoderCount();
    previous_time_ = micros();

    pid_input_ = 0;
    pid_output_ = 0;
    pid_setpoint_ = 0;
  }

  void update() {
    long current_count = encoder_.getEncoderCount();
    pid_setpoint_ = target_position_;
    pid_input_ = current_count;

    pid_->Compute();

    if (stop_) {
      setMotorVelocity(0);
      return;
    }

    setMotorVelocity(int(pid_output_));
  }

  void print() {
    Serial.print("Input ");
    Serial.print(pid_input_);
    Serial.print(", Output ");
    Serial.print(pid_output_);
    Serial.print(", Setpoint ");
    Serial.print(pid_setpoint_);
    Serial.print(", ControlOutput ");
    Serial.println(last_motor_velocity_);
  }

  void setPosition(float position) { target_position_ = position; }

  void stop() { stop_ = true; }

 private:
  void setMotorVelocity(int velocity) {
    if (velocity < -255) velocity = -255;
    if (velocity > 255) velocity = 255;

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

  Adafruit_MotorShield afms_;
  Encoders encoder_;
  bool stop_;
  const int count_per_revolution_;

  Adafruit_DCMotor* motor_;
  PID* pid_;
  long previous_count_;
  long previous_time_;

  float target_position_;
  int last_motor_velocity_;

  double pid_input_, pid_output_, pid_setpoint_;
  int print_count_;
};

PIDControl* g_PIDControl;

PIDControl* classic_pid(int pin_a, int pin_b, int motor, float ku, float tu) {
  float kp = ku * 0.6;
  float ki = 1.2 * ku / tu;
  float kd = 3.0 * ku * tu / 40.0;
  return new PIDControl(pin_a, pin_b, motor, ENCODER_COUNT_PER_REV, kp, ki, kd);
}

PIDControl* pessen_pid(int pin_a, int pin_b, int motor, float ku, float tu) {
  float kp = ku * 0.7;
  float ki = 1.75 * ku / tu;
  float kd = 21.0 * ku * tu / 200.0;
  return new PIDControl(pin_a, pin_b, motor, ENCODER_COUNT_PER_REV, kp, ki, kd);
}

PIDControl* damped_pid(int pin_a, int pin_b, int motor, float ku, float tu) {
  float kp = ku * 0.2;
  float ki = 0.4 * ku / tu;
  float kd = ku * tu / 15.0;
  return new PIDControl(pin_a, pin_b, motor, ENCODER_COUNT_PER_REV, kp, ki, kd);
}

void setup() {
  Serial.begin(115200);
  g_PIDControl = damped_pid(2, 3, 1, 3.2, 0.12);
  g_PIDControl =
      new PIDControl(2, 3, 1, ENCODER_COUNT_PER_REV, .6f, 10.0f, 0.00f);
  delay(2000);
}

void loop() {
  float p;
  if (millis() < 5000) {
    p = -4800;
  } else if (millis() < 10000) {
    p = 4800;
  }

  if (millis() > 15000) {
    g_PIDControl->stop();
    g_PIDControl->update();
  } else {
    g_PIDControl->setPosition(p);
    g_PIDControl->update();
    g_PIDControl->print();
  }
  while (micros() % 10000 > 0) {
  }
}
