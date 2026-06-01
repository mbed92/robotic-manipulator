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

void Pca9685ServoDriver::setRamp(uint16_t step_us, uint16_t step_delay_ms) {
    config_.ramp_step_us = step_us;
    config_.ramp_step_delay_ms = step_delay_ms;
}

void Pca9685ServoDriver::setServoUs(uint8_t channel, uint16_t target_us) {
    if (channel >= CHANNEL_COUNT) {
        return;
    }

    if (!ensureCurrentPwmKnown(channel)) {
        // If the current PWM is unknown, jump immediately to the target without ramping.
        writeServoUs(channel, target_us);
        return;
    } else {
        // Ramp from the current PWM to the target PWM in steps, with delays in between.
        int32_t current_us = current_pwm_us_[channel];
        const int32_t target = target_us;
        const int32_t direction = (target >= current_us) ? 1 : -1;
        const int32_t step = (config_.ramp_step_us == 0) ? 1 : config_.ramp_step_us;

        while (current_us != target) {
            const int32_t next_us = current_us + direction * step;

            if ((direction > 0 && next_us > target) || (direction < 0 && next_us < target)) {
                current_us = target;
            } else {
                current_us = next_us;
            }

            writeServoUs(channel, static_cast<uint16_t>(current_us));
            delay(config_.ramp_step_delay_ms);
        }
    }
}

void Pca9685ServoDriver::setServoUsImmediate(uint8_t channel, uint16_t target_us) {
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

bool Pca9685ServoDriver::ensureCurrentPwmKnown(uint8_t channel) {
    if (current_pwm_known_[channel]) {
        return true;
    }

    // PCA9685 can report the PWM register value it currently outputs.
    // This is only the last stored command in the driver chip, not measured joint position.
    const uint16_t current_ticks = pca_.getPWM(channel, true);
    if (current_ticks > 0 && current_ticks < TICKS_PER_PERIOD) {
        current_pwm_us_[channel] = ticksToPwmUs(current_ticks);
        current_pwm_known_[channel] = true;
        return true;
    }

    // No useful previous command is available, so avoid inventing a fake start point.
    // The caller decides how to handle the first command after this unknown state.
    return false;
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

uint16_t Pca9685ServoDriver::ticksToPwmUs(uint16_t ticks) {
    return static_cast<uint16_t>(
        (static_cast<uint32_t>(ticks) * SERVO_PWM_PERIOD_US) / TICKS_PER_PERIOD);
}
