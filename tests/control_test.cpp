#include "Module/gimbal.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

using gimbal::Controller;
using gimbal::Output;
using gimbal::Sample;
using gimbal::State;

namespace {
constexpr float period = 0.010f;
constexpr Sample level{0, 0, 1, 0, 0, 0};

void check(bool condition, const char* description) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", description);
        std::exit(1);
    }
}

struct Rig {
    Controller controller;
    std::uint32_t now = 0;

    Output tick(Sample sample = level) {
        now += 10;
        return controller.step(sample, period, now);
    }

    void calibrate(Sample still = level) {
        for (int i = 0; i < 220; ++i) {
            check(!tick(still).enabled, "outputs disabled during calibration and idle");
        }
        check(controller.state() == State::Ready, "stationary samples complete calibration");
    }

    void arm() {
        controller.heartbeat(now);
        check(controller.arm(now), "fresh heartbeat allows explicit arming");
        check(tick().enabled && controller.state() == State::Active, "active outputs");
    }
};

void calibrationAndAttitude() {
    Rig rig;
    check(!rig.controller.arm(0), "cannot arm at startup");
    for (int i = 0; i < 150; ++i) rig.tick();
    Sample moving = level;
    moving.gz = 25;
    for (int i = 0; i < 220; ++i) rig.tick(moving);
    for (int i = 0; i < 100; ++i) rig.tick();
    check(rig.controller.state() == State::Calibrating, "motion restarts the stillness window");

    Rig biased;
    Sample still = level;
    still.gz = 1;
    biased.calibrate(still);
    Output out;
    for (int i = 0; i < 500; ++i) out = biased.tick(still);
    check(std::fabs(out.yaw) < 0.1f, "estimated stationary gyro bias removed");
    moving = still;
    moving.gz += 30;
    for (int i = 0; i < 100; ++i) out = biased.tick(moving);
    check(std::fabs(out.yaw - 30) < 0.5f, "relative yaw integrates a known positive turn");
    check(std::fabs(out.roll) < 0.2f && std::fabs(out.pitch) < 0.2f, "yaw turn preserves level axes");

    Rig inclined;
    constexpr float sine30 = 0.5f;
    constexpr float cosine30 = 0.8660254f;
    const Sample tilted{-sine30, 0, cosine30, 0, 0, 0};
    inclined.calibrate(tilted);
    out = inclined.tick(tilted);
    check(std::fabs(out.pitch - 30) < 0.5f, "initial tilt is referenced to gravity");

    Rig translating;
    for (int i = 0; i < 400; ++i) {
        const Sample translated{i % 2 == 0 ? 0.25f : -0.25f, 0, 0.9682458f, 0, 0, 0};
        translating.tick(translated);
    }
    check(translating.controller.state() == State::Calibrating, "unstable acceleration rejects calibration");
    translating.calibrate();
}

void linkAndFaultLatch() {
    Rig rig;
    rig.now = std::numeric_limits<std::uint32_t>::max() - 2450U;
    rig.calibrate();
    check(!rig.controller.arm(rig.now), "fresh boot needs radio heartbeat");
    rig.arm();
    // The arming heartbeat was one tick ago. This crosses the 32-bit clock wrap.
    for (int i = 0; i < 48; ++i) check(rig.tick().enabled, "healthy before link timeout");
    check(!rig.tick().enabled && rig.controller.state() == State::Fault, "heartbeat expires at 500 ms");
    rig.controller.heartbeat(rig.now);
    rig.controller.disarm();
    check(!rig.controller.arm(rig.now) && !rig.tick().enabled, "reconnect and stop leave fault latched");
    check(rig.controller.reset(), "operator can clear fault");
    rig.calibrate();
    check(!rig.controller.arm(rig.now), "reset requires another fresh heartbeat");
}

void commandsAndLimits() {
    Rig rig;
    rig.calibrate();
    rig.arm();
    check(rig.controller.target(10, -10), "accept in-range two-axis command");
    check(!rig.controller.target(35.1f, 2) && !rig.controller.target(2, -35.1f),
          "reject mechanical target range violations");
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    check(!rig.controller.target(invalid, 2), "reject non-finite setpoint");
    check(rig.controller.targetYaw() == 10 && rig.controller.targetPitch() == -10,
          "rejected frames preserve both targets");
    check(rig.controller.setGain(2), "accept finite filter gain");
    check(!rig.controller.setGain(invalid) && !rig.controller.setGain(-1) &&
          !rig.controller.setGain(10.1f) && rig.controller.gain() == 2,
          "invalid gains do not change live filter");
    const auto first = rig.tick();
    check(first.enabled && std::fabs(first.yawServo - 90) < 3 && std::fabs(first.pitchServo - 90) < 3,
          "first commanded step is rate-limited");
    Output out = first;
    for (int i = 0; i < 40; ++i) {
        out = rig.tick();
        check(out.enabled && out.yawServo >= 0 && out.yawServo <= 180 &&
              out.pitchServo >= 0 && out.pitchServo <= 180, "bounded servo angles");
    }
    check(out.yawServo > 90 && out.pitchServo < 90, "commands drive the two axes independently");
    rig.controller.disarm();
    check(!rig.tick().enabled && rig.controller.state() == State::Ready, "operator stop disables outputs");
}

void travelAndTracking() {
    for (const float direction : {-1.f, 1.f}) {
        Rig rig;
        rig.calibrate();
        rig.arm();
        check(rig.controller.target(direction * 35, -direction * 35), "supported travel boundary");
        Output out;
        for (int i = 0; i < 140 && rig.controller.state() == State::Active; ++i) {
            if (i % 10 == 0) rig.controller.heartbeat(rig.now);
            out = rig.tick();
            check(out.yawServo >= 45 && out.yawServo <= 135 &&
                  out.pitchServo >= 45 && out.pitchServo <= 135, "enforce both nominal servo stops");
            if (i == 90) {
                check(out.enabled && out.yawServo == 90 + direction * 45 &&
                      out.pitchServo == 90 - direction * 45, "largest request reaches both output clamps");
            }
            if (i < 120) check(out.enabled, "allow transient tracking error");
        }
        check(!out.enabled && rig.controller.state() == State::Fault, "sustained tracking error trips");
    }
}

void smallErrorSaturation() {
    for (const float direction : {-1.f, 1.f}) {
        Rig rig;
        rig.calibrate();
        rig.arm();
        check(rig.controller.target(direction * 4, -direction * 4), "small two-axis target accepted");
        Output out;
        for (int i = 0; i < 500 && rig.controller.state() == State::Active; ++i) {
            if (i % 10 == 0) rig.controller.heartbeat(rig.now);
            out = rig.tick();
        }
        check(rig.controller.state() == State::Fault && !out.enabled,
              "small sustained error at either output limit disables power");
    }
}

void smoothRearm() {
    Rig rig;
    rig.calibrate();
    rig.arm();
    check(rig.controller.target(4, -4), "rearm test setpoint");
    Output before;
    for (int i = 0; i < 60; ++i) {
        if (i % 10 == 0) rig.controller.heartbeat(rig.now);
        before = rig.tick();
    }
    check(before.yawServo > 100 && before.pitchServo < 80, "start away from the nominal centers");
    rig.controller.disarm();
    rig.arm();
    auto after = rig.tick();
    check(after.yawServo == before.yawServo && after.pitchServo == before.pitchServo,
          "STOP and ARM preserve the last position command");
    rig.controller.disarm();
    check(rig.controller.reset(), "stopped controller accepts reset");
    rig.calibrate();
    rig.arm();
    after = rig.tick();
    check(after.yawServo == before.yawServo && after.pitchServo == before.pitchServo,
          "software reset retains last position command through recalibration");
}

void liveGain() {
    Rig rig;
    rig.calibrate();
    check(rig.controller.setGain(0), "zero gain selectable");
    const Sample tilted{-0.5f, 0, 0.8660254f, 0, 0, 0};
    Output out;
    for (int i = 0; i < 200; ++i) out = rig.tick(tilted);
    check(std::fabs(out.pitch) < 0.1f, "zero gain suppresses accelerometer correction");
    check(rig.controller.setGain(5), "update gain without restarting");
    for (int i = 0; i < 200; ++i) out = rig.tick(tilted);
    check(std::fabs(out.pitch - 30) < 0.3f, "online gain corrects pitch toward gravity");
}

void sampleAndTimingFaults() {
    Rig imu;
    imu.calibrate();
    imu.arm();
    Sample bad = level;
    bad.gx = std::numeric_limits<float>::quiet_NaN();
    check(!imu.tick(bad).enabled && imu.controller.state() == State::Fault, "invalid sensor disables outputs");
    check(!imu.tick().enabled, "valid readings cannot clear a latched fault");

    Rig timer;
    timer.calibrate();
    timer.arm();
    check(!timer.controller.step(level, 0.15f, timer.now + 150).enabled &&
          timer.controller.state() == State::Fault, "late control cycle disables outputs");
}
}

int main() {
    calibrationAndAttitude();
    linkAndFaultLatch();
    commandsAndLimits();
    travelAndTracking();
    smallErrorSaturation();
    smoothRearm();
    liveGain();
    sampleAndTimingFaults();
    std::puts("control OK");
}
