#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f3xx_hal.h"

typedef struct Motor {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    GPIO_TypeDef      *son_port;
    uint16_t           son_pin;
    bool               son_active_low;
    bool               dir_invert;
    volatile uint32_t  steps_remaining;
    volatile bool      moving;
    /* TaskHandle_t (opaque here). If non-NULL, motor IRQ calls
     * vTaskNotifyGiveFromISR on this task when the last step is generated. */
    void              *notify_task;
} Motor;

void motor_init(Motor *m,
                TIM_HandleTypeDef *htim,
                uint32_t channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin,
                GPIO_TypeDef *son_port, uint16_t son_pin);

void motor_enable(Motor *m);
void motor_disable(Motor *m);
void motor_set_dir(Motor *m, bool positive);

bool motor_move(Motor *m, uint32_t steps, uint32_t hz);

/* Start two channels of the same timer atomically (e.g. X dual-drive on
 * TIM4 CH1+CH2). a->htim must equal b->htim. */
bool motor_move_pair(Motor *a, Motor *b, uint32_t steps, uint32_t hz);

bool motor_is_moving(const Motor *m);

void motor_set_notify_task(Motor *m, void *task_handle);

/* Call from HAL_TIM_PeriodElapsedCallback for every TIM update event. */
void motor_irq_dispatch(TIM_HandleTypeDef *htim);

#endif
