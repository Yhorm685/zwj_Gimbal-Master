#include "Module/mpu6050.hpp"
#include "BSP/i2c.hpp"
#include <atomic>
#include <array>

namespace fw::mpu6050 {
namespace {
std::atomic<uint8_t> address{0};

int16_t word(const uint8_t* bytes) {
    const int32_t value = (static_cast<int32_t>(bytes[0]) << 8) | bytes[1];
    return static_cast<int16_t>(value < 32768 ? value : value - 65536);
}
}

Status initialize(TransferMode mode) {
    address.store(0);
    bool mismatched = false;
    for (const uint8_t candidate : std::array<uint8_t, 2>{0x68, 0x69}) {
        uint8_t id{};
        auto rc = i2c::read(candidate, 0x75, &id, 1, mode);
        if (rc == Status::BusError) continue;  // Unpopulated AD0 address usually NACKs.
        if (rc != Status::Ok) return rc;
        if (id != 0x68) { mismatched = true; continue; }
        const uint8_t clock = 0x01;  // Wake up, PLL on X gyro.
        rc = i2c::write(candidate, 0x6B, &clock, 1, mode);
        if (rc != Status::Ok) return rc;
        // SMPLRT_DIV=7, DLPF_CFG=3, FS_SEL=0, AFS_SEL=0.
        constexpr std::array<uint8_t, 4> config{7, 3, 0, 0};
        rc = i2c::write(candidate, 0x19, config.data(), config.size(), mode);
        if (rc != Status::Ok) return rc;
        std::array<uint8_t, 4> actual{};
        rc = i2c::read(candidate, 0x19, actual.data(), actual.size(), mode);
        if (rc != Status::Ok) return rc;
        if (actual != config) return Status::DeviceMismatch;
        address.store(candidate);
        return Status::Ok;
    }
    return mismatched ? Status::DeviceMismatch : Status::BusError;
}

Status sample(Sample& output, TransferMode mode) {
    const auto device = address.load();
    if (!device) return Status::NotInitialized;
    uint8_t buf[14];
    const auto rc = i2c::read(device, 0x3B, buf, sizeof buf, mode);
    if (rc != Status::Ok) return rc;
    output = {word(buf), word(buf+2), word(buf+4), word(buf+6), word(buf+8), word(buf+10), word(buf+12)};
    return Status::Ok;
}

int32_t accelMilliG(int16_t raw) { return static_cast<int32_t>(raw) * 1000 / 16384; }
int32_t gyroMilliDps(int16_t raw) { return static_cast<int32_t>(raw) * 1000 / 131; }
}
