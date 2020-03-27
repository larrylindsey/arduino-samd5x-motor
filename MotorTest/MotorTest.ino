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

    previous_count_ = encoder_.getEncoderCount();
    previous_time_ = micros();

    pid_input_ = 0;
    pid_output_ = 0;
    pid_setpoint_ = 0;
  }


  void update() {
    long current_count = encoder_.getEncoderCount();
    long current_time = micros();
    float time_delta = float(current_time - previous_time_) / 1000000.0;
    float measured_velocity = float(current_count - previous_count_) /
                              count_per_revolution_ / time_delta;
    previous_count_ = current_count;
    previous_time_ = current_time;

    pid_setpoint_ = target_velocity_;
    pid_input_ = measured_velocity;

    pid_->Compute();

    if (stop_) {
      motorVelocity_(0);
      return;
    }

    int control_velocity = int(pid_output_ / V_MAX_REV_PER_S * 255);
    if (control_velocity < 0) control_velocity = 0;
    if (control_velocity > 255) control_velocity = 255;
    motorVelocity_(control_velocity);
  }

  void print() {
    Serial.print(", Input ");
    Serial.print(pid_input_);
    Serial.print(", Output ");
    Serial.print(pid_output_);
    Serial.print(", Setpoint ");
    Serial.println(pid_setpoint_);
  }

  void setVelocity(float velocity) { target_velocity_ = velocity; }

  void stop() { stop_ = true; }

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
  bool stop_;
  const int count_per_revolution_;

  Adafruit_DCMotor* motor_;
  PID* pid_;
  long previous_count_;
  long previous_time_;

  float target_velocity_;
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

void setup() {
  Serial.begin(115200);
  g_PIDControl = pessen_pid(2, 3, 1, 0.6, 0.25);
  delay(2000);
}

void loop() {
  float v;
  if (millis() < 5000) {
    v = 1.0;
  } else if (millis() < 10000) {
    v = 0.5;
  }

  if (millis() > 15000) {
    g_PIDControl->stop();
  }

  g_PIDControl->setVelocity(v);
  g_PIDControl->update();
  g_PIDControl->print();

  while (micros() % 10000 > 0) {
  }
}
