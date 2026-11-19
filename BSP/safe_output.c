#include "BSP/safe_output.h"
#include "main.h"

void Gimbal_CutPower(void)
{
    /* PB12 enables the external servo rail. Its external pull-down covers reset. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->BRR = GPIO_PIN_12;
    if (__HAL_RCC_TIM3_IS_CLK_ENABLED()) {
        TIM3->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC4E);
    }
}

void Gimbal_Panic(void)
{
    __disable_irq();
    Gimbal_CutPower();
    for (;;) {}
}
