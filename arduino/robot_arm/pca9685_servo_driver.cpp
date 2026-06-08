#include "pca9685_servo_driver.h"

Pca9685ServoDriver::Pca9685ServoDriver(const Pca9685ServoDriverConfig &config)
    : pca_(),
      config_(config),
      current_pwm_us_(),
      current_pwm_known_() {
}

bool Pca9685ServoDriver::begin() {
    const bool ok = pca_.begin();
    if (!ok) {
        return false;
    }

    pca_.setPWMFreq(config_.servo_frequency_hz);
    return true;
}

void Pca9685ServoDriver::setServoUs(uint8_t channel, uint16_t target_us) {
    if (channel >= CHANNEL_COUNT) {
        return;
    }

    writeServoUs(channel, target_us);
}

bool Pca9685ServoDriver::currentServoUs(uint8_t channel, uint16_t &pwm_us) const {
    if (channel >= CHANNEL_COUNT || !current_pwm_known_[channel]) {
        return false;
    }

    pwm_us = current_pwm_us_[channel];
    return true;
}

void Pca9685ServoDriver::writeServoUs(uint8_t channel, uint16_t pwm_us) {
    pca_.setPWM(channel, 0, pwmUsToTicks(pwm_us));
    current_pwm_us_[channel] = pwm_us;
    current_pwm_known_[channel] = true;
}

uint16_t Pca9685ServoDriver::pwmUsToTicks(uint16_t pwm_us) {
    uint32_t ticks = (static_cast<uint32_t>(pwm_us) * TICKS_PER_PERIOD) / SERVO_PWM_PERIOD_US;
    if (ticks >= TICKS_PER_PERIOD) {
        ticks = TICKS_PER_PERIOD - 1;
    }
    return static_cast<uint16_t>(ticks);
}
