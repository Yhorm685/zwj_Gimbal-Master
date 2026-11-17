#pragma once

#include "usart.h"
#include "Common/status.hpp"
#include <array>
#include <cstddef>

namespace fw {
// One Comm task owns USART1. RX and TX can be in flight independently.
class Uart {
public:
    explicit Uart(UART_HandleTypeDef& handle) : handle_(handle) {}
    ~Uart();
    Uart(const Uart&) = delete;
    Uart& operator=(const Uart&) = delete;
    static constexpr std::size_t capacity = 128;
    void setWakeup(Wakeup fn, void* context);
    Status write(const std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs);
    Status read(std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs);
    Status startWrite(const std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs);
    Status startRead(std::size_t size, std::uint32_t timeoutMs);
    Status pollWrite();
    Status pollRead(std::uint8_t* out, std::size_t capacity);
    Status cancelWrite();
    Status cancelRead();
    Status startStream();
    Status pop(std::uint8_t& byte, std::uint32_t& arrivedMs);
    Status stopStream();
    std::uint32_t streamOverruns() const { return overruns_; }

    static void completeFromIrq(UART_HandleTypeDef* handle, bool receive, Status result);
private:
    struct Transfer {
        std::array<std::uint8_t, capacity> bytes{};
        std::uint32_t started{};
        std::uint32_t timeout{};
        std::uint16_t size{};
        volatile Status result{Status::Busy};
        bool owned{};
    };
    Status start(bool receive, const std::uint8_t* data, std::size_t size, std::uint32_t timeoutMs);
    Status finish(bool receive, bool cancel, std::uint8_t* out, std::size_t size);
    bool ready() const;
    void receiveStreamFromIrq(Status result);
    struct Sample {
        std::uint32_t time;
        std::uint8_t byte;
    };
    static constexpr std::size_t ringSize = 64;
    std::array<Sample, ringSize> ring_{};
    std::size_t head_{}, tail_{};
    std::uint8_t nextByte_{};
    bool streaming_{};
    volatile Status streamFault_{Status::Ok};
    volatile std::uint32_t overruns_{};
    UART_HandleTypeDef& handle_;
    Transfer tx_{};
    Transfer rx_{};
    Wakeup wake_{};
    void* context_{};
    static Uart* instance_;
};
}
