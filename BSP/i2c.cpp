#include "BSP/i2c.hpp"
#include "BSP/f103_transfer.hpp"
#include "i2c.h"
#include <cstring>

extern "C" osSemaphoreId_t i2cDoneHandle;

namespace fw::i2c {
namespace {
std::atomic_flag gate = ATOMIC_FLAG_INIT;
detail::Operation operation;
detail::CompletionSignal completion{i2cDoneHandle};
uint8_t storage[64];
size_t readSize{};

void notifyCompletion(I2C_HandleTypeDef* handle) {
    if (handle == &hi2c1) completion.notify();
}

bool valid(uint8_t address, size_t size, uint32_t ms) {
    return address >= 0x08 && address <= 0x77 && size && size <= sizeof(storage) && detail::validTimeout(ms);
}

Status restore() {
    operation.active = false;
    if (hi2c1.Instance != I2C1) return Status::NotInitialized;
    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);
    const auto stopped = HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    if (stopped != HAL_OK) return detail::status(stopped);
    return detail::status(HAL_I2C_Init(&hi2c1));
}

Status mapped(HAL_StatusTypeDef rc) {
    if (rc == HAL_ERROR && (HAL_I2C_GetError(&hi2c1) & HAL_I2C_ERROR_TIMEOUT)) return Status::Timeout;
    return detail::status(rc);
}

Status ready() {
    if (hi2c1.Instance != I2C1 || HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET) return Status::NotInitialized;
    return operation.active ? Status::Busy : Status::Ok;
}

Status run(bool reading, uint8_t address, uint8_t reg, uint8_t* data, size_t size, uint32_t ms, Ticket* ticket) {
    detail::Gate lock(gate);
    if (!lock) return Status::Busy;
    auto result = ready();
    if (result != Status::Ok) return result;
    const uint16_t addr = static_cast<uint16_t>(address << 1);
    const uint16_t n = static_cast<uint16_t>(size);
    HAL_StatusTypeDef rc;
    if (ticket) {
        if (!completion.reset()) return Status::NotInitialized;
        if (!reading) std::memcpy(storage, data, size);
        operation.arm(*ticket, ms);
        readSize = reading ? size : 0;
        rc = reading ? HAL_I2C_Mem_Read_IT(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, storage, n)
                     : HAL_I2C_Mem_Write_IT(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, storage, n);
    } else {
        rc = reading ? HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, n, ms)
                     : HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, n, ms);
    }
    result = mapped(rc);
    if (rc != HAL_OK) {
        operation.active = false;
        // A NACK is a completed transaction; a stuck BUSY/timeout needs reinitialization.
        if (result == Status::Busy || result == Status::Timeout) {
            const auto recovered = restore();
            if (recovered != Status::Ok) return recovered;
        }
    }
    return result;
}
}

Status startRead(uint8_t address, uint8_t reg, size_t size, Ticket& ticket, uint32_t ms) {
    if (!valid(address, size, ms)) return Status::InvalidArgument;
    return run(true, address, reg, nullptr, size, ms, &ticket);
}
Status startWrite(uint8_t address, uint8_t reg, const uint8_t* data, size_t size, Ticket& ticket, uint32_t ms) {
    if (!data || !valid(address, size, ms)) return Status::InvalidArgument;
    return run(false, address, reg, const_cast<uint8_t*>(data), size, ms, &ticket);
}
Status wait(Ticket ticket, uint8_t* output, size_t capacity) {
    detail::Gate lock(gate);
    if (!lock) return Status::Busy;
    if (!operation.owns(ticket)) return Status::Cancelled;
    if (readSize && (!output || capacity < readSize)) return Status::InvalidArgument;
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        if (operation.timedOut()) {
            const auto recovered = restore();
            return recovered == Status::Ok ? Status::Timeout : recovered;
        }
        completion.wait(operation.start, operation.duration);
    }
    operation.active = false;
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) return mapped(HAL_ERROR);
    if (readSize) std::memcpy(output, storage, readSize);
    return Status::Ok;
}
namespace {
Status finish(Ticket ticket, uint8_t* output, size_t size) {
    Status result;
    do {
        result = wait(ticket, output, size);
        if (result == Status::Busy) detail::nextTick();
    } while (result == Status::Busy);
    return result;
}
}
Status read(uint8_t address, uint8_t reg, uint8_t* data, size_t size, TransferMode mode, uint32_t ms) {
    if (!data || !valid(address, size, ms)) return Status::InvalidArgument;
    if (mode == TransferMode::Blocking) return run(true, address, reg, data, size, ms, nullptr);
    Ticket ticket;
    const auto status = startRead(address, reg, size, ticket, ms);
    return status == Status::Ok ? finish(ticket, data, size) : status;
}
Status write(uint8_t address, uint8_t reg, const uint8_t* data, size_t size, TransferMode mode, uint32_t ms) {
    if (!data || !valid(address, size, ms)) return Status::InvalidArgument;
    if (mode == TransferMode::Blocking) return run(false, address, reg, const_cast<uint8_t*>(data), size, ms, nullptr);
    Ticket ticket;
    const auto status = startWrite(address, reg, data, size, ticket, ms);
    return status == Status::Ok ? finish(ticket, nullptr, 0) : status;
}
Status cancel(Ticket ticket) {
    detail::Gate lock(gate);
    if (!lock) return Status::Busy;
    if (!operation.owns(ticket)) return Status::Cancelled;
    return restore();
}
Status recover() {
    detail::Gate lock(gate);
    return lock ? restore() : Status::Busy;
}
}

extern "C" {
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* handle) { fw::i2c::notifyCompletion(handle); }
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* handle) { fw::i2c::notifyCompletion(handle); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* handle) { fw::i2c::notifyCompletion(handle); }
}
