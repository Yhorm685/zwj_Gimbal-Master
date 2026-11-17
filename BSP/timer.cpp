#include "BSP/timer.hpp"
#include "BSP/pwm.hpp"
#include "BSP/irq_guard.hpp"
#include "BSP/hal_status.hpp"

namespace fw {
Timer* Timer::active_{};
Timer::~Timer() { stopPeriodic(); }

Status Timer::waitUpdate(std::uint32_t timeoutMs) {
    if (!detail::validTimeout(timeoutMs)) return Status::InvalidArgument;
    if (!handle_.Instance || handle_.State == HAL_TIM_STATE_RESET) return Status::NotInitialized;
    if (active_ == this || handle_.State != HAL_TIM_STATE_READY) return Status::Busy;
    __HAL_TIM_SET_COUNTER(&handle_, 0);
    __HAL_TIM_CLEAR_FLAG(&handle_, TIM_FLAG_UPDATE);
    const auto start = HAL_TIM_Base_Start(&handle_);
    if (start != HAL_OK) return detail::halStatus(start);
    const auto since = HAL_GetTick();
    Status result = Status::Ok;
    while (!__HAL_TIM_GET_FLAG(&handle_, TIM_FLAG_UPDATE)) {
        if (HAL_GetTick() - since >= timeoutMs) {
            result = Status::Timeout;
            break;
        }
    }
    const auto stop = HAL_TIM_Base_Stop(&handle_);
    __HAL_TIM_CLEAR_FLAG(&handle_, TIM_FLAG_UPDATE);
    return stop == HAL_OK ? result : detail::halStatus(stop);
}

Status Timer::startPeriodic(Wakeup fn, void* context) {
    if (!handle_.Instance || handle_.State == HAL_TIM_STATE_RESET) return Status::NotInitialized;
    detail::IrqGuard guard;
    if (active_ || handle_.State != HAL_TIM_STATE_READY) return Status::Busy;
    wake_ = fn;
    context_ = context;
    ticks_ = 0;
    __HAL_TIM_SET_COUNTER(&handle_, 0);
    __HAL_TIM_CLEAR_FLAG(&handle_, TIM_FLAG_UPDATE);
    const auto start = HAL_TIM_Base_Start_IT(&handle_);
    if (start == HAL_OK) active_ = this;
    return detail::halStatus(start);
}

Status Timer::stopPeriodic() {
    detail::IrqGuard guard;
    if (active_ != this) return Status::NotInitialized;
    const auto stop = HAL_TIM_Base_Stop_IT(&handle_);
    if (stop == HAL_OK) {
        __HAL_TIM_CLEAR_FLAG(&handle_, TIM_FLAG_UPDATE);
        active_ = nullptr;
    }
    return detail::halStatus(stop);
}

void Timer::elapsedFromIrq(TIM_HandleTypeDef* handle) {
    if (!active_ || &active_->handle_ != handle) return;
    ++active_->ticks_;
    if (active_->wake_) active_->wake_(active_->context_);
}
}

extern "C" void Framework_TimerElapsed(TIM_HandleTypeDef* handle) {
    fw::Timer::elapsedFromIrq(handle);
    fw::PwmChannel::elapsedFromIrq(handle);
}
