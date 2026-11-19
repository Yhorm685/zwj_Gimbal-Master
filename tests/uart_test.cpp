#include "BSP/uart.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
unsigned checks{};
void check(bool condition, const char* description) {
    ++checks;
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", description);
        std::exit(1);
    }
}

struct MockUart {
    UART_HandleTypeDef handle{this, HAL_UART_STATE_READY, HAL_UART_STATE_READY, 0};
    std::uint8_t* rx{};
    const std::uint8_t* tx{};
    std::uint16_t rxSize{}, txSize{};
    unsigned rxArms{}, txArms{}, rxAborts{}, txAborts{}, wakes{};
    HAL_StatusTypeDef rxReply{HAL_OK}, txReply{HAL_OK}, abortReply{HAL_OK};

    void receive(std::uint8_t byte, std::uint32_t tick) {
        check(rx != nullptr && rxSize == 1, "mock receives only into an armed one-byte buffer");
        *rx = byte;
        rx = nullptr;
        testTick = tick;
        HAL_UART_RxCpltCallback(&handle);
    }
    void transmitted() {
        check(tx != nullptr, "mock completes only an active transmission");
        tx = nullptr;
        HAL_UART_TxCpltCallback(&handle);
    }
};

MockUart& mock(UART_HandleTypeDef* handle) {
    return *static_cast<MockUart*>(handle->Instance);
}
void wake(void* context) { ++static_cast<MockUart*>(context)->wakes; }
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef* handle, const std::uint8_t*,
                                   std::uint16_t, std::uint32_t) {
    return mock(handle).txReply;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef* handle, std::uint8_t*,
                                  std::uint16_t, std::uint32_t) {
    return mock(handle).rxReply;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef* handle, const std::uint8_t* data,
                                     std::uint16_t size) {
    auto& port = mock(handle);
    ++port.txArms;
    if (port.txReply != HAL_OK) return port.txReply;
    if (port.tx) return HAL_BUSY;
    port.tx = data;
    port.txSize = size;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef* handle, std::uint8_t* data,
                                    std::uint16_t size) {
    auto& port = mock(handle);
    ++port.rxArms;
    if (port.rxReply != HAL_OK) return port.rxReply;
    if (port.rx) return HAL_BUSY;
    port.rx = data;
    port.rxSize = size;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef* handle) {
    auto& port = mock(handle);
    ++port.rxAborts;
    if (port.abortReply == HAL_OK) port.rx = nullptr;
    return port.abortReply;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef* handle) {
    auto& port = mock(handle);
    ++port.txAborts;
    if (port.abortReply == HAL_OK) port.tx = nullptr;
    return port.abortReply;
}

namespace {
using fw::Status;

void interleavedStreamsAndTransmit() {
    MockUart usb, radio, unknown;
    fw::Uart first{usb.handle}, second{radio.handle};
    first.setWakeup(wake, &usb);
    second.setWakeup(wake, &radio);
    check(first.startStream() == Status::Ok && second.startStream() == Status::Ok,
          "both independent handles can stream simultaneously");
    std::uint8_t a[]{1, 2}, b[]{3, 4}, byte{};
    std::uint32_t arrived{};
    check(first.startWrite(a, 2, 100) == Status::Ok &&
          second.startWrite(b, 2, 100) == Status::Ok, "both streaming ports can transmit");
    a[0] = b[0] = 99;
    check(usb.tx != radio.tx && usb.tx[0] == 1 && radio.tx[0] == 3,
          "each TX owns its own copied payload");
    HAL_UART_RxCpltCallback(&unknown.handle);
    HAL_UART_TxCpltCallback(&unknown.handle);
    HAL_UART_ErrorCallback(nullptr);
    check(usb.wakes == 0 && radio.wakes == 0, "unknown callbacks wake neither owner");
    usb.receive(11, 10);
    radio.receive(22, 11);
    usb.receive(33, 12);
    check(usb.wakes == 2 && radio.wakes == 1, "RX callbacks select the matching wakeup context");
    check(first.pop(byte, arrived) == Status::Ok && byte == 11 && arrived == 10,
          "first stream retains its first byte and timestamp");
    check(second.pop(byte, arrived) == Status::Ok && byte == 22 && arrived == 11,
          "second stream retains its own byte and timestamp");
    check(first.pop(byte, arrived) == Status::Ok && byte == 33 && arrived == 12,
          "interleaving preserves first stream order");
    check(second.pop(byte, arrived) == Status::Busy, "first stream adds no bytes to second stream");
    radio.transmitted();
    check(second.pollWrite() == Status::Ok && first.pollWrite() == Status::Busy,
          "second TX callback completes only second transfer");
    usb.transmitted();
    check(first.pollWrite() == Status::Ok && usb.wakes == 3 && radio.wakes == 2,
          "first TX callback completes only first transfer");

    radio.receive(44, 13);
    usb.receive(55, 14);
    const auto beforeError = usb.rxArms;
    usb.handle.ErrorCode = 8;
    usb.receive(66, 15);
    check(usb.rxArms == beforeError, "RX complete with pending HAL error does not rearm early");
    HAL_UART_ErrorCallback(&usb.handle);
    check(first.pop(byte, arrived) == Status::BusError && usb.rxArms == beforeError + 1 &&
          usb.handle.ErrorCode == 0, "first stream aborts and rearms after its error");
    check(first.pop(byte, arrived) == Status::Busy, "fault discards first stream's stale buffered data");
    check(second.pop(byte, arrived) == Status::Ok && byte == 44 && radio.rxAborts == 0,
          "first stream error preserves second buffer and active receive");
    usb.receive(77, 16);
    check(first.pop(byte, arrived) == Status::Ok && byte == 77, "first stream recovers independently");

    for (unsigned i = 0; i < 64; ++i) usb.receive(static_cast<std::uint8_t>(i), 20 + i);
    check(first.pop(byte, arrived) == Status::Overflow && first.streamOverruns() == 1,
          "first ring overflow is reported");
    check(second.streamOverruns() == 0 && second.pop(byte, arrived) == Status::Busy,
          "first overflow does not contaminate second stream");
    radio.receive(88, 90);
    check(second.pop(byte, arrived) == Status::Ok && byte == 88, "second stream survives peer overflow");
}

void transferTimeoutAndErrorIsolation() {
    testTick = UINT32_MAX - 5;
    MockUart usb, radio;
    fw::Uart first{usb.handle}, second{radio.handle};
    std::uint8_t byte{};
    check(first.startRead(1, 10) == Status::Ok && second.startRead(1, 20) == Status::Ok,
          "both handles can own finite RX transactions");
    testTick = 5;
    check(first.pollRead(&byte, 1) == Status::Timeout && usb.rxAborts == 1,
          "first timeout works across the millisecond counter wrap");
    check(second.pollRead(&byte, 1) == Status::Busy && radio.rxAborts == 0,
          "first timeout leaves second receive active");
    radio.receive(42, 6);
    check(second.pollRead(&byte, 1) == Status::Ok && byte == 42, "second read still completes");
    check(first.startRead(1, 20) == Status::Ok && second.startRead(1, 20) == Status::Ok,
          "both reads can restart");
    usb.receive(7, 7);
    HAL_UART_ErrorCallback(&usb.handle);
    check(first.pollRead(&byte, 1) == Status::BusError, "RX error wins over same-IRQ completion");
    check(second.pollRead(&byte, 1) == Status::Busy, "RX error is isolated to matching handle");
    check(second.cancelRead() == Status::Cancelled && radio.rxAborts == 1, "cancel aborts matching HAL RX");

    usb.rxReply = HAL_BUSY;
    check(first.startRead(1, 20) == Status::Busy, "HAL start rejection is propagated");
    usb.rxReply = HAL_OK;
    check(first.startRead(1, 20) == Status::Ok, "failed start releases transaction ownership");
    usb.abortReply = HAL_ERROR;
    check(first.cancelRead() == Status::BusError && first.startRead(1, 20) == Status::Busy,
          "failed abort preserves ownership until the hardware is detached");
    usb.abortReply = HAL_OK;
    check(first.cancelRead() == Status::Cancelled, "retry can finish an earlier failed abort");
}

void bindingOwnershipAndReuse() {
    testTick = 0;
    MockUart usb, radio, spare;
    fw::Uart first{usb.handle}, third{spare.handle};
    check(first.startStream() == Status::Ok, "first registry slot is available after earlier destructors");
    {
        fw::Uart duplicate{usb.handle};
        std::uint8_t byte{};
        check(duplicate.startStream() == Status::Busy &&
              duplicate.startWrite(&byte, 1, 10) == Status::Busy,
              "a second wrapper cannot steal the same HAL handle");
    }
    usb.receive(9, 1);
    std::uint8_t byte{};
    std::uint32_t arrived{};
    check(first.pop(byte, arrived) == Status::Ok && byte == 9,
          "destroying an unbound duplicate preserves the registered owner");
    {
        fw::Uart second{radio.handle};
        check(second.startStream() == Status::Ok, "second registry slot is available");
        check(second.startWrite(&byte, 1, 10) == Status::Ok, "second owner has active RX and TX");
        check(third.startStream() == Status::Busy && spare.rxArms == 0,
              "registry exhaustion does not start an unrouteable HAL receive");
    }
    check(radio.rx == nullptr && radio.tx == nullptr &&
          radio.rxAborts == 1 && radio.txAborts == 1,
          "destruction synchronously detaches both transfers");
    check(third.startStream() == Status::Ok, "destroyed owner's registry slot is reusable");
    third.setWakeup(wake, &spare);
    HAL_UART_RxCpltCallback(&radio.handle);
    HAL_UART_TxCpltCallback(&radio.handle);
    HAL_UART_ErrorCallback(&radio.handle);
    check(spare.wakes == 0, "late callbacks for destroyed owner cannot reach replacement handle");
    spare.receive(10, 2);
    check(third.pop(byte, arrived) == Status::Ok && byte == 10 && spare.wakes == 1,
          "replacement handle receives its own callback");
    usb.receive(11, 3);
    check(first.pop(byte, arrived) == Status::Ok && byte == 11,
          "remaining registered owner continues after peer destruction");
}
}

int main() {
    interleavedStreamsAndTransmit();
    transferTimeoutAndErrorIsolation();
    bindingOwnershipAndReuse();
    check(testMask == 0, "all IRQ guards restore the caller's interrupt mask");
    std::printf("PASS dual UART: %u checks (mock HAL)\n", checks);
}
