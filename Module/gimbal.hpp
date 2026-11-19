#pragma once
#include <cstdint>

namespace gimbal {
struct Sample { float ax, ay, az, gx, gy, gz; }; // g and degrees/s in platform coordinates
enum class State { Calibrating, Ready, Active, Fault };
struct Output { float roll{}, pitch{}, yaw{}, yawServo{90}, pitchServo{90}; bool enabled{}; };

class Controller {
public:
    Controller();
    Output step(Sample sample, float dt, std::uint32_t nowMs);
    bool arm(std::uint32_t nowMs);
    void disarm();
    bool reset();
    void fault();
    void heartbeat(std::uint32_t nowMs);
    bool target(float yaw, float pitch);
    bool setGain(float gain);
    float gain() const { return gain_; }
    State state() const { return state_; }
    float targetYaw() const { return yawTarget_; }
    float targetPitch() const { return pitchTarget_; }
private:
    bool calibrate(Sample s);
    bool updateAttitude(Sample s, float dt);
    State state_{State::Calibrating};
    Output out_{};
    float gain_{1.5f};
    float q_[4]{1,0,0,0};
    float bias_[3]{};
    float sum_[6]{};
    float sampleM2_[6]{};
    unsigned stillCount_{};
    float yawTarget_{}, pitchTarget_{};
    float trackingTime_{};
    float saturationTime_{};
    std::uint32_t lastBeat_{};
    bool haveBeat_{};
};
}
