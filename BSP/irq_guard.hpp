#pragma once

// Include a board HAL header before this private helper.
namespace fw::detail {
class IrqGuard {
public:
    IrqGuard() : mask_(__get_PRIMASK()) { __disable_irq(); }
    ~IrqGuard() { __set_PRIMASK(mask_); }
    IrqGuard(const IrqGuard&) = delete;
    IrqGuard& operator=(const IrqGuard&) = delete;
private:
    unsigned mask_;
};
}
