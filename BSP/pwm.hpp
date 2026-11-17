#pragma once

#include "tim.h"
#include "Common/status.hpp"

namespace fw {
// One timer tick = 1 us. CCR preload must be enabled in CubeMX.
// A timer's PWM channels share its update flag. Confirm one pulse change at a time.
class PwmChannel {
public:
    PwmChannel(TIM_HandleTypeDef& handle, std::uint32_t channel) : handle_(handle), channel_(channel) {}
    ~PwmChannel();
    PwmChannel(const PwmChannel&) = delete;
    PwmChannel& operator=(const PwmChannel&) = delete;
    Status start(std::uint16_t pulseUs);
    Status stop();
    Status setPulseUs(std::uint16_t pulseUs);
    Status waitPulseUs(std::uint16_t pulseUs, std::uint32_t timeoutMs);
    Status startPulseUs(std::uint16_t pulseUs, std::uint32_t timeoutMs, Wakeup fn = nullptr, void* ctx = nullptr);
    Status pollPulse();
    static void elapsedFromIrq(TIM_HandleTypeDef* handle);
private:
    Status validate(std::uint16_t pulseUs) const;
    TIM_HandleTypeDef& handle_;
    std::uint32_t channel_;
    bool running_{};
    bool waiting_{};
    volatile bool completed_{};
    volatile std::uint32_t completedAt_{};
    std::uint32_t since_{};
    std::uint32_t timeout_{};
    Wakeup wake_{};
    void* context_{};
    static PwmChannel* owner_;
};
}
