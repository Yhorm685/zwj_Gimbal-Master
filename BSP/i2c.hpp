#pragma once

#include "BSP/transfer_mode.hpp"
#include "Common/status.hpp"
#include <cstddef>

namespace fw::i2c {
// 7-bit address, 8-bit register, 1..64 bytes. All calls are task-context only.
Status read(uint8_t address, uint8_t reg, uint8_t* data, size_t size, TransferMode mode, uint32_t ms = 30);
Status write(uint8_t address, uint8_t reg, const uint8_t* data, size_t size, TransferMode mode, uint32_t ms = 30);
Status startRead(uint8_t address, uint8_t reg, size_t size, Ticket& ticket, uint32_t ms = 30);
Status startWrite(uint8_t address, uint8_t reg, const uint8_t* data, size_t size, Ticket& ticket, uint32_t ms = 30);
// Reads copy into output on success; writes pass nullptr, 0.
Status wait(Ticket ticket, uint8_t* output, size_t capacity);
Status cancel(Ticket ticket);
Status recover();
}
