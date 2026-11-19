#include "BSP/board_timer.hpp"

namespace fw::board {
Timer& clockTimer() {
    static Timer timer{htim2};
    return timer;
}
}
