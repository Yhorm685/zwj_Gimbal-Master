#pragma once

#include <cstdint>

namespace fw {
enum class TransferMode { Blocking, Interrupt };
struct Ticket { std::uint32_t value{}; };
}
