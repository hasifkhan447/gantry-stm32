#include "solenoid.h"

#include "stm32f3xx_hal.h"

#define SOL_PORT   GPIOB
#define SOL_PIN    GPIO_PIN_0

static bool s_state;

void solenoid_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(SOL_PORT, SOL_PIN, GPIO_PIN_RESET);

    GPIO_InitTypeDef g = {0};
    g.Pin   = SOL_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SOL_PORT, &g);

    s_state = false;
}

void solenoid_set(bool on)
{
    HAL_GPIO_WritePin(SOL_PORT, SOL_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_state = on;
}

bool solenoid_toggle(void)
{
    solenoid_set(!s_state);
    return s_state;
}

bool solenoid_get(void)
{
    return s_state;
}
