#pragma once
// Minimal host HAL adapted from task 1 tests/support/test_hal.hpp.
#include <cstdint>

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
constexpr std::uint32_t HAL_MAX_DELAY = UINT32_MAX;
inline std::uint32_t testMask{}, testTick{};
inline std::uint32_t __get_PRIMASK() { return testMask; }
inline void __disable_irq() { testMask = 1; }
inline void __set_PRIMASK(std::uint32_t mask) { testMask = mask; }
inline std::uint32_t HAL_GetTick() { return testTick; }
