#ifndef ROBOT_ARM_PCA9685_SERVO_DRIVER_H
#define ROBOT_ARM_PCA9685_SERVO_DRIVER_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

struct Pca9685ServoDriverConfig {
    uint16_t servo_frequency_hz;
    uint16_t ramp_step_us;
    uint16_t ramp_step_delay_ms;
};

class Pca9685ServoDriver {
  public:
    static constexpr uint8_t CHANNEL_COUNT = 16;
    static constexpr uint16_t SERVO_PWM_PERIOD_US = 20000;
    static constexpr uint16_t TICKS_PER_PERIOD = 4096;

    explicit Pca9685ServoDriver(
        const Pca9685ServoDriverConfig &config = {50, 10, 20});

    bool begin();
    void setRamp(uint16_t step_us, uint16_t step_delay_ms);
    void setServoUs(uint8_t channel, uint16_t target_us);
    void setServoUsImmediate(uint8_t channel, uint16_t target_us);
    bool currentServoUs(uint8_t channel, uint16_t &pwm_us) const;

  private:
    Adafruit_PWMServoDriver pca_;
    Pca9685ServoDriverConfig config_;
    uint16_t current_pwm_us_[CHANNEL_COUNT];
    bool current_pwm_known_[CHANNEL_COUNT];

    bool ensureCurrentPwmKnown(uint8_t channel);
    void writeServoUs(uint8_t channel, uint16_t pwm_us);
    static uint16_t pwmUsToTicks(uint16_t pwm_us);
    static uint16_t ticksToPwmUs(uint16_t ticks);
};

#endif
