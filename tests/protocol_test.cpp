#include "Application/protocol.hpp"
#include "Module/gimbal.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

using namespace gimbal;

static void check(bool ok, const char* description) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", description); std::exit(1); }
}

static Command read(LineReader& reader, const char* text) {
    Command cmd;
    int completed = 0;
    while (*text) if (reader.feed(*text++, cmd)) ++completed;
    check(completed == 1, "one complete line");
    return cmd;
}

int main() {
    LineReader usb, radio;
    Command cmd;
    check(!usb.feed('G', cmd), "partial USB frame");
    check(read(radio, "  TARGET -12.25 +.5\r\n").action == Action::Target, "separate readers");
    auto gain = read(usb, "AIN 1.25\n");
    check(gain.action == Action::Gain && gain.first == 1.25f, "split frame");
    for (auto line : {"ARM junk\n", "GET\rX\n", "GAIN .\n", "TARGET 1 2 3\n",
                      "TARGET 1\n", "GAIN 1e3\n", "PING\r\r\n", "GAIN NaN\n"})
        check(read(usb, line).action == Action::Invalid, "reject malformed input");
    for (int i = 0; i < 70; ++i) check(!usb.feed(' ', cmd), "discard overlong frame");
    check(read(usb, "ARM\n").action == Action::Invalid, "reject suffix of oversized frame");
    usb.discardUntilNewline();
    check(read(usb, "ARM\n").action == Action::Invalid, "serial overrun loses frame");
    check(read(usb, "PING\n").action == Action::Ping, "recover at delimiter");
    Controller ctl;
    auto denied = apply(gain, Port::Bluetooth, ctl, 1);
    check(!denied.ok, "radio cannot change gain");
    auto accepted = apply(gain, Port::Usb, ctl, 2);
    check(accepted.ok && accepted.parameter, "USB gain acknowledged");
    check(!apply(read(usb, "PING\n"), Port::Usb, ctl, 3).ok, "USB cannot maintain radio heartbeat");
    check(!apply(read(usb, "ARM\n"), Port::Usb, ctl, 4).ok, "USB cannot arm");
    check(apply(read(radio, "STOP\n"), Port::Bluetooth, ctl, 5).ok, "radio stop");
    char data[160];
    check(formatReply(data, sizeof(data), gain, accepted, ctl.gain()) > 0 &&
          std::strcmp(data, "OK GAIN\nPARAM 1.250\n") == 0, "acknowledge applied value");
    check(formatReply(data, 6, gain, accepted, ctl.gain()) == 0, "suppress truncated frame");
    Output o{};
    o.roll = -0.125f;
    check(formatTelemetry(data, sizeof(data), 40, o, ctl, 0, 150) > 0 &&
          std::strncmp(data, "T,40,-0.125,", 12) == 0 && std::strstr(data, ",1.250,0,150\n"),
          "telemetry matches host field layout");
    std::puts("protocol OK");
}
