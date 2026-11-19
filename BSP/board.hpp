#pragma once
#include "BSP/uart.hpp"

namespace fw::board {
Uart& console();
uint32_t millis();
const char* name();
}
