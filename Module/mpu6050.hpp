#pragma once

#include "BSP/transfer_mode.hpp"
#include "Common/status.hpp"

namespace fw::mpu6050 {
struct Sample { int16_t ax, ay, az, temperature, gx, gy, gz; };
Status initialize(TransferMode mode);
Status sample(Sample& output, TransferMode mode);
int32_t accelMilliG(int16_t raw);
int32_t gyroMilliDps(int16_t raw);
}
