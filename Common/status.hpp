#pragma once

#include <cstdint>

namespace fw {
enum class Status : std::uint8_t {
    Ok, NotInitialized, InvalidArgument, Busy, Timeout, BusError,
    ChecksumError, DeviceMismatch, Overflow, Cancelled
};

constexpr const char* statusName(Status value) {
    switch (value) {
    case Status::Ok: return "Ok";
    case Status::NotInitialized: return "NotInitialized";
    case Status::InvalidArgument: return "InvalidArgument";
    case Status::Busy: return "Busy";
    case Status::Timeout: return "Timeout";
    case Status::BusError: return "BusError";
    case Status::ChecksumError: return "ChecksumError";
    case Status::DeviceMismatch: return "DeviceMismatch";
    case Status::Overflow: return "Overflow";
    case Status::Cancelled: return "Cancelled";
    }
    return "Unknown";
}

using Wakeup = void (*)(void*);
}
