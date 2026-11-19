#include "Module/gimbal.hpp"
#include <algorithm>
#include <cmath>

namespace gimbal {
namespace {
constexpr float radians = 0.01745329252f;
constexpr float degrees = 57.29577951f;
constexpr unsigned calibrationSamples = 200;
constexpr float targetLimit = 35.0f;

struct ServoCalibration { float center, low, high, direction; };
// Nominal unloaded values. Measure both joints before replacing these limits.
constexpr ServoCalibration yawAxis{90, 45, 135, 1};
constexpr ServoCalibration pitchAxis{90, 45, 135, 1};

float wrap(float angle) {
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}

float drive(float previous, float error, float rate, float dt, const ServoCalibration& axis, bool& saturated) {
    // The position loop asks for angular velocity; the rate loop drives the
    // built-in SG90 position servo by integrating bounded position increments.
    const float wantedRate = std::clamp(4.0f * error, -65.0f, 65.0f);
    const float commandRate = std::clamp(wantedRate + 0.45f * (wantedRate - rate), -70.0f, 70.0f);
    const float requested = previous + dt * axis.direction * commandRate;
    saturated = requested < axis.low || requested > axis.high;
    return std::clamp(requested, axis.low, axis.high);
}

bool valid(Sample s) {
    return std::isfinite(s.ax) && std::isfinite(s.ay) && std::isfinite(s.az) &&
           std::isfinite(s.gx) && std::isfinite(s.gy) && std::isfinite(s.gz) &&
           std::fabs(s.gx) < 249 && std::fabs(s.gy) < 249 && std::fabs(s.gz) < 249;
}
}

Controller::Controller() {
    out_.yawServo = yawAxis.center;
    out_.pitchServo = pitchAxis.center;
}

bool Controller::calibrate(Sample s) {
    const float length2 = s.ax*s.ax + s.ay*s.ay + s.az*s.az;
    if (std::fabs(length2 - 1) > 0.10f ||
        std::max({std::fabs(s.gx), std::fabs(s.gy), std::fabs(s.gz)}) > 3) {
        stillCount_ = 0;
        return false;
    }
    const float values[] = {s.ax, s.ay, s.az, s.gx, s.gy, s.gz};
    if (stillCount_ == 0) {
        std::fill_n(sum_, 6, 0.0f);
        std::fill_n(sampleM2_, 6, 0.0f);
    }
    ++stillCount_;
    for (unsigned i = 0; i < 6; ++i) {
        const float delta = values[i] - sum_[i];
        sum_[i] += delta / static_cast<float>(stillCount_); // running mean
        sampleM2_[i] += delta * (values[i] - sum_[i]);
    }
    if (stillCount_ < calibrationSamples) return false;
    // A steady gyro alone also occurs during translation. Require stable
    // acceleration components as well as a plausible gravity magnitude.
    const float count = static_cast<float>(stillCount_ - 1);
    const float accelVariance = *std::max_element(sampleM2_, sampleM2_ + 3) / count;
    const float gyroVariance = *std::max_element(sampleM2_ + 3, sampleM2_ + 6) / count;
    if (accelVariance > 0.0025f || gyroVariance > 0.25f) {
        stillCount_ = 0;
        return false;
    }
    const float roll = std::atan2(sum_[1], sum_[2]);
    const float pitch = std::atan2(-sum_[0], std::hypot(sum_[1], sum_[2]));
    const float cr = std::cos(roll/2), sr = std::sin(roll/2);
    const float cp = std::cos(pitch/2), sp = std::sin(pitch/2);
    q_[0] = cr*cp; q_[1] = sr*cp; q_[2] = cr*sp; q_[3] = -sr*sp;
    std::copy_n(sum_ + 3, 3, bias_);
    out_.roll = roll * degrees;
    out_.pitch = pitch * degrees;
    out_.yaw = 0;
    return true;
}

bool Controller::updateAttitude(Sample s, float dt) {
    float gx = (s.gx - bias_[0]) * radians;
    float gy = (s.gy - bias_[1]) * radians;
    float gz = (s.gz - bias_[2]) * radians;
    float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    const float norm2 = s.ax*s.ax + s.ay*s.ay + s.az*s.az;
    if (!std::isfinite(norm2) || norm2 < 0.01f) return false;
    if (norm2 >= 0.81f && norm2 <= 1.21f) {
        const float inv = 1 / std::sqrt(norm2);
        const float ax = s.ax*inv, ay = s.ay*inv, az = s.az*inv;
        const float vx = 2*(x*z - w*y), vy = 2*(w*x + y*z), vz = 1 - 2*(x*x+y*y);
        gx += gain_ * (ay*vz - az*vy);
        gy += gain_ * (az*vx - ax*vz);
        gz += gain_ * (ax*vy - ay*vx);
    }
    const float h = dt * 0.5f;
    q_[0] = w + h*(-x*gx-y*gy-z*gz);
    q_[1] = x + h*( w*gx+y*gz-z*gy);
    q_[2] = y + h*( w*gy-x*gz+z*gx);
    q_[3] = z + h*( w*gz+x*gy-y*gx);
    const float n = std::sqrt(q_[0]*q_[0]+q_[1]*q_[1]+q_[2]*q_[2]+q_[3]*q_[3]);
    if (!std::isfinite(n) || n < 0.1f) return false;
    for (float& v : q_) v /= n;
    w = q_[0]; x = q_[1]; y = q_[2]; z = q_[3];
    out_.roll  = degrees * std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
    out_.pitch = degrees * std::asin(std::clamp(2*(w*y-z*x), -1.0f, 1.0f));
    out_.yaw   = degrees * std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    return true;
}

Output Controller::step(Sample s, float dt, std::uint32_t nowMs) {
    if (state_ == State::Fault) return out_;
    if (!std::isfinite(dt) || dt < 0.004f || dt > 0.025f || !valid(s)) {
        fault();
        return out_;
    }
    if (state_ == State::Calibrating) {
        if (calibrate(s)) state_ = State::Ready;
        return out_;
    }
    if (!updateAttitude(s, dt)) { fault(); return out_; }
    if (state_ != State::Active) return out_;
    if (!haveBeat_ || nowMs - lastBeat_ >= 500 || std::fabs(out_.pitch) >= 65) {
        fault();
        return out_;
    }
    const float yawError = wrap(yawTarget_ - out_.yaw);
    const float pitchError = pitchTarget_ - out_.pitch;
    if (std::max(std::fabs(yawError), std::fabs(pitchError)) > 5) trackingTime_ += dt;
    else trackingTime_ = 0;
    if (trackingTime_ >= 1.25f) { fault(); return out_; }
    const float sr = std::sin(out_.roll*radians), cr = std::cos(out_.roll*radians);
    const float gy = s.gy - bias_[1], gz = s.gz - bias_[2];
    const float pitchRate = cr*gy - sr*gz;
    const float yawRate = (sr*gy + cr*gz) / std::cos(out_.pitch*radians);
    bool yawSaturated, pitchSaturated;
    out_.yawServo = drive(out_.yawServo, yawError, yawRate, dt, yawAxis, yawSaturated);
    out_.pitchServo = drive(out_.pitchServo, pitchError, pitchRate, dt, pitchAxis, pitchSaturated);
    if (yawSaturated || pitchSaturated) saturationTime_ += dt;
    else saturationTime_ = 0;
    if (saturationTime_ >= 1.25f) fault();
    return out_;
}

void Controller::heartbeat(std::uint32_t nowMs) { lastBeat_ = nowMs; haveBeat_ = true; }

bool Controller::arm(std::uint32_t nowMs) {
    if (state_ != State::Ready || !haveBeat_ || nowMs - lastBeat_ >= 500 ||
        std::fabs(out_.yaw) > targetLimit || std::fabs(out_.pitch) > targetLimit) return false;
    yawTarget_ = out_.yaw;
    pitchTarget_ = out_.pitch;
    // Retain the last command across STOP/ARM. On first boot the mechanism
    // must be aligned with the nominal center before power is enabled.
    trackingTime_ = 0;
    saturationTime_ = 0;
    state_ = State::Active;
    out_.enabled = true;
    return true;
}

void Controller::disarm() {
    out_.enabled = false;
    haveBeat_ = false;
    if (state_ == State::Active) state_ = State::Ready;
}

void Controller::fault() { disarm(); state_ = State::Fault; }

bool Controller::reset() {
    if (state_ == State::Active) return false;
    const float previousGain = gain_;
    const float previousYaw = out_.yawServo;
    const float previousPitch = out_.pitchServo;
    *this = Controller{};
    gain_ = previousGain;
    out_.yawServo = previousYaw;
    out_.pitchServo = previousPitch;
    return true;
}

bool Controller::target(float yaw, float pitch) {
    if (state_ != State::Active || !std::isfinite(yaw) || !std::isfinite(pitch) ||
        std::fabs(yaw) > targetLimit || std::fabs(pitch) > targetLimit) return false;
    yawTarget_ = yaw;
    pitchTarget_ = pitch;
    return true;
}

bool Controller::setGain(float gain) {
    if (!std::isfinite(gain) || gain < 0 || gain > 10) return false;
    gain_ = gain;
    return true;
}
}
