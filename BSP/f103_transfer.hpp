#pragma once

#include "BSP/transfer_mode.hpp"
#include "Common/status.hpp"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <atomic>

namespace fw::detail {
// Task-side gates never spin. Interrupt handlers only publish hardware results.
class Gate {
    std::atomic_flag& flag_;
    bool acquired_;
public:
    explicit Gate(std::atomic_flag& flag) : flag_(flag), acquired_(!flag.test_and_set()) {}
    ~Gate() { if (acquired_) flag_.clear(); }
    explicit operator bool() const { return acquired_; }
    Gate(const Gate&) = delete;
    Gate& operator=(const Gate&) = delete;
};

inline Status status(HAL_StatusTypeDef result) {
    switch (result) {
        case HAL_OK: return Status::Ok;
        case HAL_BUSY: return Status::Busy;
        case HAL_TIMEOUT: return Status::Timeout;
        default: return Status::BusError;
    }
}

inline bool validTimeout(uint32_t ms) { return ms > 0 && ms <= INT32_MAX; }
inline bool expired(uint32_t start, uint32_t duration) { return HAL_GetTick() - start >= duration; }
inline void nextTick() { vTaskDelay(1); }

// The caller checks the peripheral's final state after every wakeup.
class CompletionSignal {
    const osSemaphoreId_t& handle_;
public:
    explicit constexpr CompletionSignal(const osSemaphoreId_t& handle) : handle_(handle) {}

    // Called under the peripheral gate, before enabling an interrupt transfer.
    bool reset() const {
        if (!handle_) return false;
        const auto result = osSemaphoreAcquire(handle_, 0);
        return result == osOK || result == osErrorResource;
    }

    void notify() const {
        if (handle_) (void)osSemaphoreRelease(handle_);
    }

    void wait(uint32_t startMs, uint32_t timeoutMs) const {
        const uint32_t elapsed = HAL_GetTick() - startMs;
        if (elapsed >= timeoutMs) return;
        static_assert(configTICK_RATE_HZ == 1000, "F103 completion waits use one RTOS tick per ms");
        (void)osSemaphoreAcquire(handle_, timeoutMs - elapsed);
    }
};

// Each peripheral has its own ticket sequence. Call under the peripheral gate.
struct Operation {
    uint32_t sequence{}, start{}, duration{};
    bool active{};
    void arm(Ticket& ticket, uint32_t ms) {
        if (++sequence == 0) ++sequence;
        ticket.value = sequence;
        duration = ms;
        start = HAL_GetTick();
        active = true;
    }
    bool owns(Ticket ticket) const { return active && ticket.value == sequence; }
    bool timedOut() const { return expired(start, duration); }
};
}
