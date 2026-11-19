#include "BSP/uart.hpp"
#include "BSP/hal_status.hpp"
#include "BSP/irq_guard.hpp"
#include <cstring>

namespace fw {
using detail::IrqGuard;
using detail::halStatus;
std::array<Uart*, 2> Uart::instances_{};

// Caller holds the IRQ guard until the HAL operation has been started.
bool Uart::bind() {
    for (auto* port : instances_) {
        if (port && &port->handle_ == &handle_) return port == this;
    }
    for (auto& port : instances_) {
        if (!port) { port = this; return true; }
    }
    return false;
}

Uart::~Uart() {
    stopStream();
    cancelWrite();
    cancelRead();
    IrqGuard guard;
    for (auto& port : instances_) if (port == this) port = nullptr;
}

bool Uart::ready() const {
    return handle_.Instance != nullptr && handle_.gState != HAL_UART_STATE_RESET &&
           handle_.RxState != HAL_UART_STATE_RESET;
}

void Uart::setWakeup(Wakeup fn, void* context) {
    IrqGuard guard;
    wake_ = fn;
    context_ = context;
}

Status Uart::write(const std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs) {
    if (!data || !size || size > UINT16_MAX || !detail::validTimeout(timeoutMs)) return Status::InvalidArgument;
    if (!ready()) return Status::NotInitialized;
    if (tx_.owned) return Status::Busy;
    return halStatus(HAL_UART_Transmit(&handle_, const_cast<std::uint8_t*>(data),
                                      static_cast<std::uint16_t>(size), timeoutMs));
}

Status Uart::read(std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs) {
    if (!data || !size || size > UINT16_MAX || !detail::validTimeout(timeoutMs)) return Status::InvalidArgument;
    if (!ready()) return Status::NotInitialized;
    if (rx_.owned || streaming_) return Status::Busy;
    const auto result = HAL_UART_Receive(&handle_, data, static_cast<std::uint16_t>(size), timeoutMs);
    if (result == HAL_TIMEOUT && handle_.ErrorCode != HAL_UART_ERROR_NONE) return Status::BusError;
    return halStatus(result);
}

Status Uart::startWrite(const std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs) {
    if (!data) return Status::InvalidArgument;
    return start(false, data, size, timeoutMs);
}

Status Uart::startRead(std::size_t size, std::uint32_t timeoutMs) {
    return start(true, nullptr, size, timeoutMs);
}

Status Uart::start(bool receive, const std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs) {
    if (!size || size > capacity || !detail::validTimeout(timeoutMs)) return Status::InvalidArgument;
    if (!ready()) return Status::NotInitialized;
    IrqGuard guard;
    auto& op = receive ? rx_ : tx_;
    if (op.owned || (receive && streaming_) || !bind()) return Status::Busy;
    if (!receive) std::memcpy(op.bytes.data(), data, size);
    op.size = static_cast<std::uint16_t>(size);
    op.started = HAL_GetTick();
    op.timeout = timeoutMs;
    op.result = Status::Busy;
    op.owned = true;
    const auto result = receive ? HAL_UART_Receive_IT(&handle_, op.bytes.data(), op.size)
                                : HAL_UART_Transmit_IT(&handle_, op.bytes.data(), op.size);
    if (result != HAL_OK) op.owned = false;
    return halStatus(result);
}

Status Uart::pollWrite() { return finish(false, false, nullptr, 0); }
Status Uart::pollRead(std::uint8_t* out, std::size_t size) { return finish(true, false, out, size); }
Status Uart::cancelWrite() { return finish(false, true, nullptr, 0); }
Status Uart::cancelRead() { return finish(true, true, nullptr, 0); }

Status Uart::finish(bool receive, bool cancel, std::uint8_t* out, std::size_t size) {
    IrqGuard guard;
    auto& op = receive ? rx_ : tx_;
    if (!op.owned) return Status::NotInitialized;
    Status result = op.result;
    if (cancel) {
        result = Status::Cancelled;
    } else {
        if (receive && (!out || size < op.size)) return Status::InvalidArgument;
        if (result == Status::Busy) {
            if (HAL_GetTick() - op.started < op.timeout) return Status::Busy;
            result = Status::Timeout;
        }
    }
    if (result == Status::Ok) {
        if (receive) std::memcpy(out, op.bytes.data(), op.size);
    } else {
        // No DMA is bound. Abort synchronously detaches the HAL transfer before re-use.
        const auto aborted = receive ? HAL_UART_AbortReceive(&handle_) : HAL_UART_AbortTransmit(&handle_);
        if (aborted != HAL_OK) return halStatus(aborted);
        if (receive) __HAL_UART_CLEAR_OREFLAG(&handle_);
    }
    op.owned = false;
    return result;
}

void Uart::completeFromIrq(UART_HandleTypeDef* handle, bool receive, Status result) {
    Uart* self = nullptr;
    for (auto* port : instances_) {
        if (port && &port->handle_ == handle) { self = port; break; }
    }
    if (!self) return;
    if (receive && self->streaming_) {
        self->receiveStreamFromIrq(result);
        return;
    }
    auto& op = receive ? self->rx_ : self->tx_;
    if (!op.owned) return;
    if (result == Status::Ok && HAL_GetTick() - op.started >= op.timeout) result = Status::Timeout;
    // A recoverable RX error can share an interrupt with RX completion.
    if (op.result == Status::Busy || result == Status::BusError) op.result = result;
    if (self->wake_) self->wake_(self->context_);
}

Status Uart::startStream() {
    if (!ready()) return Status::NotInitialized;
    IrqGuard guard;
    if (streaming_ || rx_.owned || !bind()) return Status::Busy;
    head_ = tail_ = 0;
    streamFault_ = Status::Ok;
    const auto result = HAL_UART_Receive_IT(&handle_, &nextByte_, 1);
    streaming_ = result == HAL_OK;
    return halStatus(result);
}

void Uart::receiveStreamFromIrq(Status result) {
    // On F1/F4 the error IRQ may issue RX-complete before error-callback.
    // Re-arming in between would erase ErrorCode before HAL classifies it.
    if (result == Status::Ok && handle_.ErrorCode != HAL_UART_ERROR_NONE) return;
    if (result == Status::Ok) {
        const auto next = (head_ + 1) % ringSize;
        if (next == tail_) {
            ++overruns_;
            tail_ = head_;
            streamFault_ = Status::Overflow;
        }
        ring_[head_] = Sample{HAL_GetTick(), nextByte_};
        head_ = next;
        if (HAL_UART_Receive_IT(&handle_, &nextByte_, 1) != HAL_OK) result = Status::BusError;
    }
    if (result != Status::Ok) {
        streamFault_ = Status::BusError;
        head_ = tail_ = 0;
    }
    if (wake_) wake_(context_);
}

Status Uart::pop(std::uint8_t& byte, std::uint32_t& arrivedMs) {
    IrqGuard guard;
    if (!streaming_) return Status::NotInitialized;
    if (streamFault_ != Status::Ok) {
        const Status fault = streamFault_;
        if (fault == Status::BusError) {
            if (HAL_UART_AbortReceive(&handle_) != HAL_OK) return fault;
            __HAL_UART_CLEAR_OREFLAG(&handle_);
            if (HAL_UART_Receive_IT(&handle_, &nextByte_, 1) != HAL_OK) return fault;
        }
        streamFault_ = Status::Ok;
        return fault;
    }
    if (head_ == tail_) return Status::Busy;
    byte = ring_[tail_].byte;
    arrivedMs = ring_[tail_].time;
    tail_ = (tail_ + 1) % ringSize;
    return Status::Ok;
}

Status Uart::stopStream() {
    IrqGuard guard;
    if (!streaming_) return Status::NotInitialized;
    const auto result = HAL_UART_AbortReceive(&handle_);
    if (result == HAL_OK) {
        streaming_ = false;
        __HAL_UART_CLEAR_OREFLAG(&handle_);
        head_ = tail_ = 0;
    }
    return halStatus(result);
}
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* handle) {
    fw::Uart::completeFromIrq(handle, false, fw::Status::Ok);
}
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* handle) {
    fw::Uart::completeFromIrq(handle, true, fw::Status::Ok);
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* handle) {
    fw::Uart::completeFromIrq(handle, true, fw::Status::BusError);
}
