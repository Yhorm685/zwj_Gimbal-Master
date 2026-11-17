#pragma once

#include "BSP/pwm.hpp"

namespace fw {
// Conservative initial range. Calibrate endpoints after checking the mechanism.
class Sg90 {
public:
    explicit Sg90(PwmChannel& pwm, std::uint16_t minUs = 1000, std::uint16_t maxUs = 2000)
        : pwm_(pwm), minUs_(minUs), maxUs_(maxUs) {}
    Status start() {
        if (!validAngle(90)) return Status::InvalidArgument;
        return pwm_.start(pulseFor(90));
    }
    Status stop() { return pwm_.stop(); }
    Status setAngle(int degrees) {
        if (!validAngle(degrees)) return Status::InvalidArgument;
        return pwm_.setPulseUs(pulseFor(degrees));
    }
    Status setAngleBlocking(int degrees, std::uint32_t timeoutMs) {
        if (!validAngle(degrees)) return Status::InvalidArgument;
        return pwm_.waitPulseUs(pulseFor(degrees), timeoutMs);
    }
    Status startAngle(int degrees, std::uint32_t timeoutMs, Wakeup fn = nullptr, void* ctx = nullptr) {
        if (!validAngle(degrees)) return Status::InvalidArgument;
        return pwm_.startPulseUs(pulseFor(degrees), timeoutMs, fn, ctx);
    }
    Status pollAngle() { return pwm_.pollPulse(); }
private:
    bool validAngle(int degrees) const { return degrees >= 0 && degrees <= 180 && minUs_ && minUs_ < maxUs_; }
    std::uint16_t pulseFor(int degrees) const {
        return static_cast<std::uint16_t>(minUs_ + (static_cast<std::uint32_t>(degrees) * (maxUs_ - minUs_) + 90) / 180);
    }
    PwmChannel& pwm_;
    std::uint16_t minUs_, maxUs_;
};
}
