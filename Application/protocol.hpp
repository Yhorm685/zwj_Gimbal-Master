#pragma once

#include <cstddef>
#include <cstdint>

namespace gimbal {
class Controller;
struct Output;

enum class Port { Usb, Bluetooth };
enum class Action { Invalid, Ping, Arm, Stop, Reset, Target, Gain, Get };

struct Command {
    Action action{Action::Invalid};
    float first{};
    float second{};
};

// One reader per serial link. A rejected line is discarded through its LF.
class LineReader {
public:
    bool feed(char byte, Command& result);
    void reset();
    void discardUntilNewline() { invalid_ = true; }
private:
    char line_[48]{};
    std::size_t size_{};
    bool invalid_{};
    bool cr_{};
};

struct Reply { bool ok; bool parameter; };
Reply apply(const Command& command, Port port, Controller& controller, std::uint32_t nowMs);
// Return the transmitted byte count, or zero if capacity is insufficient.
int formatReply(char* buffer, std::size_t capacity, const Command& command, Reply reply, float gain);
int formatTelemetry(char* buffer, std::size_t capacity, std::uint32_t ms,
                    const Output& output, const Controller& controller,
                    std::uint32_t missed, std::uint32_t execUs);
}
