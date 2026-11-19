#include "Application/protocol.hpp"
#include "Module/gimbal.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace gimbal {
namespace {
constexpr const char* names[] = {"UNKNOWN", "PING", "ARM", "STOP", "RESET", "TARGET", "GAIN", "GET"};

void skipSpaces(const char*& p) {
    while (*p == ' ') ++p;
}

// Bounded decimal parser for the small numerical range of the serial commands.
bool number(const char*& p, float& value) {
    skipSpaces(p);
    bool negative = *p == '-';
    if (negative || *p == '+') ++p;
    bool digit = false;
    int whole = 0;
    while (*p >= '0' && *p <= '9') {
        digit = true;
        if (whole > 10000) return false;
        whole = whole * 10 + (*p++ - '0');
    }
    value = static_cast<float>(whole);
    if (*p == '.') {
        ++p;
        float place = 0.1f;
        while (*p >= '0' && *p <= '9') {
            digit = true;
            value += (*p++ - '0') * place;
            place *= 0.1f;
        }
    }
    if (negative) value = -value;
    return digit && (*p == '\0' || *p == ' ');
}

Command parse(const char* p) {
    skipSpaces(p);
    const char* start = p;
    while (*p && *p != ' ') ++p;
    const auto length = static_cast<std::size_t>(p - start);
    for (unsigned i = 1; i < sizeof(names)/sizeof(names[0]); ++i) {
        if (std::strlen(names[i]) != length || std::memcmp(start, names[i], length) != 0) continue;
        Command cmd{static_cast<Action>(i)};
        if ((cmd.action == Action::Target || cmd.action == Action::Gain) && !number(p, cmd.first)) return {};
        if (cmd.action == Action::Target && !number(p, cmd.second)) return {};
        skipSpaces(p);
        return *p == '\0' ? cmd : Command{};
    }
    return {};
}

const char* stateName(State state) {
    switch (state) {
    case State::Calibrating: return "CAL";
    case State::Ready: return "READY";
    case State::Active: return "ACTIVE";
    case State::Fault: return "FAULT";
    }
    return "FAULT";
}

// newlib-nano does not need floating-point printf support for this protocol.
struct Decimal {
    char chars[16]{};
    explicit Decimal(float value) {
        if (!std::isfinite(value) || std::fabs(value) > 100000.0f) {
            std::memcpy(chars, "nan", 4);
            return;
        }
        const auto scaled = static_cast<unsigned long>(std::fabs(value) * 1000.0f + 0.5f);
        std::snprintf(chars, sizeof(chars), "%s%lu.%03lu", (value < 0 && scaled) ? "-" : "",
                      scaled / 1000, scaled % 1000);
    }
};

int checked(int length, std::size_t capacity) {
    return length > 0 && static_cast<std::size_t>(length) < capacity ? length : 0;
}
}

bool LineReader::feed(char byte, Command& result) {
    if (byte == '\n') {
        line_[size_] = '\0';
        result = invalid_ ? Command{} : parse(line_);
        reset();
        return true;
    }
    if (byte == '\r' && !cr_) { cr_ = true; return false; }
    if (cr_ || byte < ' ' || byte > '~' || size_ == sizeof(line_) - 1) invalid_ = true;
    if (!invalid_) line_[size_++] = byte;
    return false;
}

void LineReader::reset() { size_ = 0; invalid_ = false; cr_ = false; }

Reply apply(const Command& cmd, Port port, Controller& ctl, std::uint32_t nowMs) {
    bool radio = port == Port::Bluetooth;
    switch (cmd.action) {
    case Action::Ping: if (radio) ctl.heartbeat(nowMs); return {radio, false};
    case Action::Arm: return {radio && ctl.arm(nowMs), false};
    case Action::Target: return {radio && ctl.target(cmd.first, cmd.second), false};
    case Action::Stop: ctl.disarm(); return {true, false};
    case Action::Reset: return {ctl.reset(), false};
    case Action::Gain: {
        bool ok = !radio && ctl.setGain(cmd.first);
        return {ok, ok};
    }
    case Action::Get: return {true, true};
    default: return {false, false};
    }
}

int formatReply(char* buffer, std::size_t capacity, const Command& cmd, Reply reply, float gain) {
    if (!buffer || !capacity) return 0;
    const char* name = names[static_cast<unsigned>(cmd.action)];
    int count;
    if (reply.ok && reply.parameter) {
        count = std::snprintf(buffer, capacity, "OK %s\nPARAM %s\n", name, Decimal(gain).chars);
    } else {
        count = std::snprintf(buffer, capacity, "%s %s\n", reply.ok ? "OK" : "ERR", name);
    }
    return checked(count, capacity);
}

int formatTelemetry(char* buffer, std::size_t capacity, std::uint32_t ms,
                    const Output& o, const Controller& c, std::uint32_t missed, std::uint32_t execUs) {
    if (!buffer || !capacity) return 0;
    return checked(std::snprintf(buffer, capacity, "T,%lu,%s,%s,%s,%s,%s,%s,%s,%lu,%lu\n",
        static_cast<unsigned long>(ms), Decimal(o.roll).chars, Decimal(o.pitch).chars,
        Decimal(o.yaw).chars, Decimal(c.targetYaw()).chars, Decimal(c.targetPitch()).chars,
        stateName(c.state()), Decimal(c.gain()).chars, static_cast<unsigned long>(missed),
        static_cast<unsigned long>(execUs)), capacity);
}
}
