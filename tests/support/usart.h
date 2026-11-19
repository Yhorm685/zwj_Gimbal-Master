#pragma once
// UART-only mock declarations adapted from task 1 tests/support/usart.h.
#include "test_hal.hpp"

enum { HAL_UART_STATE_RESET, HAL_UART_STATE_READY, HAL_UART_ERROR_NONE = 0 };
struct UART_HandleTypeDef {
    void* Instance;
    int gState, RxState;
    std::uint32_t ErrorCode;
};
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef*, const std::uint8_t*, std::uint16_t, std::uint32_t);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef*, std::uint8_t*, std::uint16_t, std::uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*, const std::uint8_t*, std::uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, std::uint8_t*, std::uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*);
inline void __HAL_UART_CLEAR_OREFLAG(UART_HandleTypeDef* handle) { handle->ErrorCode = HAL_UART_ERROR_NONE; }
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef*);
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
