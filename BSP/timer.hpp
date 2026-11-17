#pragma once

#include "tim.h"
#include "Common/status.hpp"

namespace fw {
// Uses the full update period configured in the .ioc. One periodic timer is bound at a time.
class Timer {
public:
    explicit Timer(TIM_HandleTypeDef& handle) : handle_(handle) {}
    ~Timer();
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;
    Status waitUpdate(std::uint32_t timeoutMs);
    Status startPeriodic(Wakeup fn = nullptr, void* context = nullptr);
    Status stopPeriodic();
    std::uint32_t ticks() const { return ticks_; }
    static void elapsedFromIrq(TIM_HandleTypeDef* handle);
private:
    TIM_HandleTypeDef& handle_;
    Wakeup wake_{};
    void* context_{};
    volatile std::uint32_t ticks_{};
    static Timer* active_;
};
}
