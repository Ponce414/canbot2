// motor_driver.h — one MDD10A channel: a PWM pin for speed, a DIR pin for direction.
//
// The MDD10A uses "sign-magnitude" control: the sign of the command picks the DIR level
// and the magnitude becomes the PWM duty. That's why the controller can output a single
// signed number per wheel and this class does the split.
#pragma once

#include <stdint.h>

class MotorDriver {
 public:
  // channel: the LEDC channel to use (0–7 on the S3). Only Arduino-ESP32 core 2.x needs
  // it; core 3.x allocates channels per pin automatically.
  bool begin(uint8_t pwm_pin, uint8_t dir_pin, bool invert, uint8_t channel);

  // duty in -1.0..+1.0, positive = "this wheel pushes the robot forward".
  void setDuty(float duty);

  float duty() const { return duty_; }

 private:
  uint8_t pwm_pin_ = 0;
  uint8_t dir_pin_ = 0;
  uint8_t channel_ = 0;
  bool invert_ = false;
  bool ready_ = false;
  float duty_ = 0.0f;
};
