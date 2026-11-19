#pragma once
#include "Common/status.hpp"
namespace fw::board {
Status servoOutput(bool enabled, float yawDegrees, float pitchDegrees);
}
