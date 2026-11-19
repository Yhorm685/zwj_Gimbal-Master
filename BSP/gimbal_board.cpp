#include "BSP/gimbal_board.hpp"
#include "BSP/pwm.hpp"
#include "BSP/safe_output.h"
#include "main.h"
#include <cmath>

namespace fw::board {
Status servoOutput(bool enabled, float yawDegrees, float pitchDegrees) {
    static PwmChannel yaw{htim3, TIM_CHANNEL_3};
    static PwmChannel pitch{htim3, TIM_CHANNEL_4};
    static bool running = false;
    if (!enabled) {
        Gimbal_CutPower();
        if (running) { yaw.stop(); pitch.stop(); running = false; }
        return Status::Ok;
    }
    if (!std::isfinite(yawDegrees) || !std::isfinite(pitchDegrees) ||
        yawDegrees < 45 || yawDegrees > 135 || pitchDegrees < 45 || pitchDegrees > 135) {
        Gimbal_CutPower();
        return Status::InvalidArgument;
    }
    const auto yawUs = static_cast<std::uint16_t>(1000 + yawDegrees * (1000.0f / 180.0f) + 0.5f);
    const auto pitchUs = static_cast<std::uint16_t>(1000 + pitchDegrees * (1000.0f / 180.0f) + 0.5f);
    const auto first = running ? yaw.setPulseUs(yawUs) : yaw.start(yawUs);
    const auto second = running ? pitch.setPulseUs(pitchUs) : pitch.start(pitchUs);
    if (first != Status::Ok || second != Status::Ok) {
        Gimbal_CutPower();
        yaw.stop();
        pitch.stop();
        running = false;
        return first != Status::Ok ? first : second;
    }
    if (!running) {
        htim3.Instance->EGR = TIM_EGR_UG;
        HAL_GPIO_WritePin(SERVO_EN_GPIO_Port, SERVO_EN_Pin, GPIO_PIN_SET);
        running = true;
    }
    return Status::Ok;
}
}
