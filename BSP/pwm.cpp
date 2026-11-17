#include "BSP/pwm.hpp"
#include "BSP/hal_status.hpp"
#include "BSP/irq_guard.hpp"

namespace fw {
PwmChannel* PwmChannel::owner_{};
PwmChannel::~PwmChannel() { stop(); }

Status PwmChannel::validate(std::uint16_t pulseUs) const {
    if (!handle_.Instance || handle_.State == HAL_TIM_STATE_RESET) return Status::NotInitialized;
    if (channel_ != TIM_CHANNEL_1 && channel_ != TIM_CHANNEL_2 &&
        channel_ != TIM_CHANNEL_3 && channel_ != TIM_CHANNEL_4) return Status::InvalidArgument;
    if (!pulseUs || pulseUs > __HAL_TIM_GET_AUTORELOAD(&handle_)) return Status::InvalidArgument;
    return Status::Ok;
}

Status PwmChannel::start(std::uint16_t pulseUs) {
    const auto validation = validate(pulseUs);
    if (validation != Status::Ok) return validation;
    if (running_) return Status::Busy;
    __HAL_TIM_SET_COMPARE(&handle_, channel_, pulseUs);
    const auto result = HAL_TIM_PWM_Start(&handle_, channel_);
    if (result == HAL_OK) running_ = true;
    return detail::halStatus(result);
}

Status PwmChannel::stop() {
    if (!running_) return Status::NotInitialized;
    detail::IrqGuard guard;
    if (owner_ == this) {
        __HAL_TIM_DISABLE_IT(&handle_, TIM_IT_UPDATE);
        owner_ = nullptr;
        waiting_ = false;
    }
    const auto result = HAL_TIM_PWM_Stop(&handle_, channel_);
    if (result == HAL_OK) running_ = false;
    return detail::halStatus(result);
}

Status PwmChannel::setPulseUs(std::uint16_t pulseUs) {
    const auto validation = validate(pulseUs);
    if (validation != Status::Ok) return validation;
    if (!running_) return Status::NotInitialized;
    if (owner_ == this) return Status::Busy;
    __HAL_TIM_SET_COMPARE(&handle_, channel_, pulseUs);
    return Status::Ok;
}

Status PwmChannel::waitPulseUs(std::uint16_t pulseUs, std::uint32_t timeoutMs) {
    if (!detail::validTimeout(timeoutMs)) return Status::InvalidArgument;
    if (owner_) return Status::Busy;
    const auto result = setPulseUs(pulseUs);
    if (result != Status::Ok) return result;
    owner_ = this;
    // Only acknowledge an update observed after writing CCR.
    __HAL_TIM_CLEAR_FLAG(&handle_, TIM_FLAG_UPDATE);
    const auto since = HAL_GetTick();
    while (!__HAL_TIM_GET_FLAG(&handle_, TIM_FLAG_UPDATE)) {
        if (HAL_GetTick() - since >= timeoutMs) {
            owner_ = nullptr;
            return Status::Timeout;
        }
    }
    owner_ = nullptr;
    return Status::Ok;
}

Status PwmChannel::startPulseUs(std::uint16_t pulseUs, std::uint32_t timeoutMs, Wakeup fn, void* ctx) {
    if (!detail::validTimeout(timeoutMs)) return Status::InvalidArgument;
    detail::IrqGuard guard;
    if (owner_) return Status::Busy;
    const auto result = setPulseUs(pulseUs);
    if (result != Status::Ok) return result;
    __HAL_TIM_CLEAR_FLAG(&handle_, TIM_FLAG_UPDATE);
    owner_ = this;
    wake_ = fn;
    context_ = ctx;
    completed_ = false;
    since_ = HAL_GetTick();
    timeout_ = timeoutMs;
    waiting_ = true;
    __HAL_TIM_ENABLE_IT(&handle_, TIM_IT_UPDATE);
    return Status::Ok;
}

void PwmChannel::elapsedFromIrq(TIM_HandleTypeDef* handle) {
    auto* self = owner_;
    if (!self || &self->handle_ != handle || !self->waiting_ || self->completed_) return;
    __HAL_TIM_DISABLE_IT(handle, TIM_IT_UPDATE);
    self->completedAt_ = HAL_GetTick();
    self->completed_ = true;
    if (self->wake_) self->wake_(self->context_);
}

Status PwmChannel::pollPulse() {
    detail::IrqGuard guard;
    if (!waiting_) return Status::NotInitialized;
    const auto elapsed = (completed_ ? completedAt_ : HAL_GetTick()) - since_;
    if (!completed_ && elapsed < timeout_) return Status::Busy;
    __HAL_TIM_DISABLE_IT(&handle_, TIM_IT_UPDATE);
    owner_ = nullptr;
    waiting_ = false;
    return elapsed < timeout_ ? Status::Ok : Status::Timeout;
}
}
