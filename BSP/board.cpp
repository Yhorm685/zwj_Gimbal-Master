#include "BSP/board.hpp"

namespace fw::board {
uint32_t millis() { return HAL_GetTick(); }

Uart& console() {
    static Uart port{huart1};
    return port;
}

const char* name() {
#if defined(FRAMEWORK_F103)
    return "F103";
#else
    return "F407";
#endif
}
}
