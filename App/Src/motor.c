#include "motor.h"

#define MOTOR_MAX        4
#define TIMER_CLK_HZ     48000000U
#define TIMER_TICK_HZ    1000000U          /* 1 us tick -> ARR is in microseconds */
#define TIMER_PRESCALER  ((TIMER_CLK_HZ / TIMER_TICK_HZ) - 1U)

static Motor *s_motors[MOTOR_MAX];
static uint8_t s_motor_count;

void motor_init(Motor *m,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin,
                GPIO_TypeDef *son_port, uint16_t son_pin)
{
    m->htim = htim;
    m->channel = channel;
    m->dir_port = dir_port;
    m->dir_pin = dir_pin;
    m->son_port = son_port;
    m->son_pin = son_pin;
    m->son_active_low = true;   /* Sigma-5 /S-ON: LOW = enabled */
    m->dir_invert = false;
    m->steps_remaining = 0;
    m->moving = false;

    motor_disable(m);
    motor_set_dir(m, true);

    if (s_motor_count < MOTOR_MAX) {
        s_motors[s_motor_count++] = m;
    }
}

static void motor_drive_son(Motor *m, bool enable)
{
    bool high = enable ^ m->son_active_low;
    HAL_GPIO_WritePin(m->son_port, m->son_pin,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_enable(Motor *m)  { motor_drive_son(m, true);  }
void motor_disable(Motor *m) { motor_drive_son(m, false); }

void motor_set_dir(Motor *m, bool positive)
{
    bool logical = m->dir_invert ? !positive : positive;
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      logical ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool motor_is_moving(const Motor *m)
{
    return m->moving;
}

bool motor_move(Motor *m, uint32_t steps, uint32_t hz)
{
    if (m == NULL || m->moving || steps == 0 || hz == 0) {
        return false;
    }

    /* With a 1 us tick, ARR = (1e6 / hz) - 1.  Clamp to 16-bit range so this
     * works on TIM3 (16-bit) as well as TIM2 (32-bit). Minimum frequency at
     * PSC=47 / 16-bit ARR is ~16 Hz, maximum is 1 MHz. */
    uint32_t arr_plus_1 = TIMER_TICK_HZ / hz;
    if (arr_plus_1 < 2U)       arr_plus_1 = 2U;
    if (arr_plus_1 > 0xFFFFU)  arr_plus_1 = 0xFFFFU;
    uint32_t arr = arr_plus_1 - 1U;

    __HAL_TIM_DISABLE(m->htim);
    __HAL_TIM_SET_PRESCALER(m->htim, TIMER_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(m->htim, arr);
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, arr_plus_1 / 2U);
    __HAL_TIM_SET_COUNTER(m->htim, 0);

    /* Force PSC/ARR shadow registers to load now, then clear the spurious
     * update flag the EGR write generated. */
    m->htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(m->htim, TIM_FLAG_UPDATE);

    m->steps_remaining = steps;
    m->moving = true;

    __HAL_TIM_ENABLE_IT(m->htim, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(m->htim, m->channel);
    return true;
}

void motor_irq_dispatch(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < s_motor_count; ++i) {
        Motor *m = s_motors[i];
        if (m->htim != htim || !m->moving) {
            continue;
        }
        if (m->steps_remaining > 0U) {
            m->steps_remaining--;
        }
        if (m->steps_remaining == 0U) {
            HAL_TIM_PWM_Stop(m->htim, m->channel);
            __HAL_TIM_DISABLE_IT(m->htim, TIM_IT_UPDATE);
            m->moving = false;
        }
    }
}
