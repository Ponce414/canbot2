#include "motor_driver.h"

#include <Arduino.h>

#include "config.h"

// The LEDC ("LED control") peripheral is the ESP32's hardware PWM generator. Once
// configured it toggles the pin on its own; the CPU only writes a new duty value.
//
// Arduino-ESP32 changed this API between major versions:
//   core 2.x: ledcSetup(channel, freq, bits); ledcAttachPin(pin, channel); ledcWrite(channel, duty)
//   core 3.x: ledcAttach(pin, freq, bits);                                 ledcWrite(pin, duty)
// The #if below picks the right calls at compile time, so the same source builds on
// whichever core your PlatformIO platform ships.

bool MotorDriver::begin(uint8_t pwm_pin, uint8_t dir_pin, bool invert, uint8_t channel) {
  pwm_pin_ = pwm_pin;
  dir_pin_ = dir_pin;
  invert_ = invert;
  channel_ = channel;

  pinMode(dir_pin_, OUTPUT);
  digitalWrite(dir_pin_, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ready_ = ledcAttach(pwm_pin_, cfg::kPwmFreqHz, cfg::kPwmBits);
#else
  ready_ = ledcSetup(channel_, cfg::kPwmFreqHz, cfg::kPwmBits) != 0;
  if (ready_) ledcAttachPin(pwm_pin_, channel_);
#endif
  setDuty(0.0f);
  return ready_;
}

void MotorDriver::setDuty(float duty) {
  if (!ready_) return;
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  duty_ = duty;

  const float physical = invert_ ? -duty : duty;  // undo a mirrored mounting
  const uint32_t max_count = (1u << cfg::kPwmBits) - 1u;
  const uint32_t counts = static_cast<uint32_t>((physical < 0 ? -physical : physical) * max_count + 0.5f);

  digitalWrite(dir_pin_, physical >= 0.0f ? HIGH : LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwm_pin_, counts);
#else
  ledcWrite(channel_, counts);
#endif
}
