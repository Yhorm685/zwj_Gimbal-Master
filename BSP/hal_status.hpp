#pragma once

#include "Common/status.hpp"

// Include a board HAL header before this private helper.
namespace fw::detail {
inline Status halStatus(HAL_StatusTypeDef result) {
    switch (result) {
    case HAL_OK: return Status::Ok;
    case HAL_BUSY: return Status::Busy;
    case HAL_TIMEOUT: return Status::Timeout;
    default: return Status::BusError;
    }
}

inline bool validTimeout(std::uint32_t timeoutMs) {
    return timeoutMs != 0 && timeoutMs != HAL_MAX_DELAY;
}
}
