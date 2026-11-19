#include "Application/runtime.h"
#include "Application/protocol.hpp"
#include "Module/gimbal.hpp"
#include "Module/mpu6050.hpp"
#include "BSP/board.hpp"
#include "BSP/board_timer.hpp"
#include "BSP/gimbal_board.hpp"
#include "BSP/i2c.hpp"
#include "BSP/irq_guard.hpp"
#include "BSP/safe_output.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "iwdg.h"
#include <atomic>
#include <cstring>

namespace {
using gimbal::Action;
using gimbal::Port;
constexpr std::uint32_t stopFlag = 1, faultFlag = 2;
constexpr unsigned commandCapacity = 8, replyCapacity = 8;
struct Request {
    gimbal::Command command;
    Port port;
    std::uint32_t arrivedMs;
};
struct Packet {
    Port port{};
    std::uint16_t length{};
    char bytes[fw::Uart::capacity]{};
};
StaticQueue_t requestControl, replyControl, telemetryControl;
std::uint8_t requestStorage[commandCapacity * sizeof(Request)];
std::uint8_t replyStorage[replyCapacity * sizeof(Packet)];
std::uint8_t telemetryStorage[sizeof(Packet)];
QueueHandle_t requests, replies, telemetry;
std::atomic<std::uint32_t> emergency{0}, completedMs{0};
std::atomic<bool> controlRunning{false};

void trip(std::uint32_t flag) {
    emergency.fetch_or(flag);
    Gimbal_CutPower();
}

void timerWake(void* context) {
    if (controlRunning.load() && HAL_GetTick() - completedMs.load() >= 30) trip(faultFlag);
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(context), &wake);
    portYIELD_FROM_ISR(wake);
}

void replyTo(const Request& request, gimbal::Reply result, float gain) {
    Packet packet;
    packet.port = request.port;
    packet.length = static_cast<std::uint16_t>(gimbal::formatReply(
        packet.bytes, sizeof(packet.bytes), request.command, result, gain));
    if (!packet.length || xQueueSend(replies, &packet, 0) != pdPASS) trip(faultFlag);
}

void receive(fw::Uart& uart, gimbal::LineReader& reader, Port port) {
    std::uint8_t byte;
    std::uint32_t arrived;
    // A bounded batch lets the second port and TX make progress under a stream.
    for (unsigned i = 0; i < 64; ++i) {
        const auto status = uart.pop(byte, arrived);
        if (status == fw::Status::Busy) return;
        if (status != fw::Status::Ok) {
            reader.discardUntilNewline();
            trip(faultFlag);
            return;
        }
        Request request{{}, port, arrived};
        if (!reader.feed(static_cast<char>(byte), request.command)) continue;
        if (request.command.action == Action::Stop) {
            trip(stopFlag);
            replyTo(request, {true, false}, 0);
        } else if (xQueueSend(requests, &request, 0) != pdPASS) {
            trip(faultFlag);
        }
    }
}

struct Transmitter {
    fw::Uart& uart;
    bool busy{};
    void poll() {
        if (!busy) return;
        const auto status = uart.pollWrite();
        if (status == fw::Status::Busy) return;
        busy = false;
        if (status != fw::Status::Ok) trip(faultFlag);
    }
    void send(const Packet& packet) {
        if (!packet.length) return;
        const auto status = uart.startWrite(
            reinterpret_cast<const std::uint8_t*>(packet.bytes), packet.length, 200);
        busy = status == fw::Status::Ok;
        if (!busy) trip(faultFlag);
    }
};
}

extern "C" void Gimbal_CreateQueues(void) {
    requests = xQueueCreateStatic(commandCapacity, sizeof(Request), requestStorage, &requestControl);
    replies = xQueueCreateStatic(replyCapacity, sizeof(Packet), replyStorage, &replyControl);
    telemetry = xQueueCreateStatic(1, sizeof(Packet), telemetryStorage, &telemetryControl);
    if (!requests || !replies || !telemetry) Gimbal_Panic();
}

extern "C" void Gimbal_Comm(void) {
    auto& usb = fw::board::console();
    fw::Uart bluetooth{huart2};
    gimbal::LineReader usbReader, bluetoothReader;
    Transmitter usbTx{usb}, bluetoothTx{bluetooth};
    if (usb.startStream() != fw::Status::Ok || bluetooth.startStream() != fw::Status::Ok) Gimbal_Panic();
    Packet pending;
    bool haveReply = false;
    for (;;) {
        receive(usb, usbReader, Port::Usb);
        receive(bluetooth, bluetoothReader, Port::Bluetooth);
        usbTx.poll();
        bluetoothTx.poll();
        if (!haveReply) haveReply = xQueueReceive(replies, &pending, 0) == pdPASS;
        if (haveReply) {
            auto& sender = pending.port == Port::Usb ? usbTx : bluetoothTx;
            if (!sender.busy) { sender.send(pending); haveReply = false; }
        }
        if (!usbTx.busy && !haveReply) {
            Packet packet;
            if (xQueueReceive(telemetry, &packet, 0) == pdPASS) usbTx.send(packet);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void StartControl(void* argument) {
    (void)argument;
    gimbal::Controller controller;
    fw::board::servoOutput(false, 90, 90);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    bool sensorReady = fw::mpu6050::initialize(fw::TransferMode::Interrupt) == fw::Status::Ok;
    if (!sensorReady) controller.fault();
    completedMs.store(HAL_GetTick());
    controlRunning.store(true);
    if (fw::board::clockTimer().startPeriodic(timerWake, xTaskGetCurrentTaskHandle()) != fw::Status::Ok) Gimbal_Panic();
    auto previousCycles = DWT->CYCCNT;
    std::uint32_t missed = 0, samples = 0;
    gimbal::Output output;
    for (;;) {
        const auto notifications = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30));
        const auto startCycles = DWT->CYCCNT;
        const auto now = HAL_GetTick();
        const float dt = static_cast<float>(startCycles - previousCycles) / SystemCoreClock;
        previousCycles = startCycles;
        if (notifications != 1) {
            missed += notifications > 1 ? notifications - 1 : 1;
            trip(faultFlag);
        }
        const auto flags = emergency.exchange(0);
        if (flags) {
            xQueueReset(requests);
            if (flags & faultFlag) controller.fault();
            else controller.disarm();
        }
        bool restarted = false;
        Request request;
        for (unsigned i = 0; i < commandCapacity && xQueueReceive(requests, &request, 0) == pdPASS; ++i) {
            gimbal::Reply reply{false, false};
            if (now - request.arrivedMs < 100) {
                // Heartbeat freshness follows the UART completion timestamp.
                const auto timestamp = request.command.action == Action::Ping ? request.arrivedMs : now;
                reply = gimbal::apply(request.command, request.port, controller, timestamp);
                if (request.command.action == Action::Reset && reply.ok) {
                    fw::board::servoOutput(false, 90, 90);
                    controlRunning.store(false);
                    if (fw::board::clockTimer().stopPeriodic() != fw::Status::Ok) Gimbal_Panic();
                    sensorReady = fw::i2c::recover() == fw::Status::Ok &&
                        fw::mpu6050::initialize(fw::TransferMode::Interrupt) == fw::Status::Ok;
                    if (!sensorReady) { controller.fault(); reply.ok = false; }
                    // Initialize with the rail off, then start a full interval.
                    ulTaskNotifyTake(pdTRUE, 0);
                    previousCycles = DWT->CYCCNT;
                    completedMs.store(HAL_GetTick());
                    if (fw::board::clockTimer().startPeriodic(timerWake, xTaskGetCurrentTaskHandle()) != fw::Status::Ok) Gimbal_Panic();
                    controlRunning.store(true);
                    restarted = true;
                }
            }
            replyTo(request, reply, controller.gain());
            if (restarted) break;
        }
        if (restarted) continue;
        fw::mpu6050::Sample raw{};
        if (sensorReady && controller.state() != gimbal::State::Fault) {
            if (fw::mpu6050::sample(raw, fw::TransferMode::Interrupt) == fw::Status::Ok) {
                const gimbal::Sample sample{
                    raw.ax / 16384.0f, raw.ay / 16384.0f, raw.az / 16384.0f,
                    raw.gx / 131.0f, raw.gy / 131.0f, raw.gz / 131.0f};
                output = controller.step(sample, dt, HAL_GetTick());
            } else {
                sensorReady = false;
                controller.fault();
            }
        }
        if ((DWT->CYCCNT - startCycles) / (SystemCoreClock / 1000000U) >= 10000) controller.fault();
        {
            // A STOP ISR or task cannot be followed by an enable from this iteration.
            fw::detail::IrqGuard guard;
            const auto pending = emergency.load();
            if (pending & faultFlag) controller.fault();
            else if (pending & stopFlag) controller.disarm();
            output.enabled = controller.state() == gimbal::State::Active && output.enabled;
            if (fw::board::servoOutput(output.enabled, output.yawServo, output.pitchServo) != fw::Status::Ok) {
                controller.fault();
                output.enabled = false;
            }
        }
        const auto executionUs = (DWT->CYCCNT - startCycles) / (SystemCoreClock / 1000000U);
        if (executionUs >= 10000) {
            controller.fault();
            fw::board::servoOutput(false, 90, 90);
            output.enabled = false;
        }
        completedMs.store(HAL_GetTick());
        if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK) Gimbal_Panic();
        if (++samples % 5 == 0) {
            Packet packet;
            packet.length = static_cast<std::uint16_t>(gimbal::formatTelemetry(
                packet.bytes, sizeof(packet.bytes), HAL_GetTick(), output, controller, missed, executionUs));
            if (packet.length) xQueueOverwrite(telemetry, &packet);
        }
    }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char*) { Gimbal_Panic(); }
extern "C" void vApplicationMallocFailedHook(void) { Gimbal_Panic(); }
