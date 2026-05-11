#include "axes.h"

#include "main.h"
#include "tim.h"

static Motor s_motor_y;
/* TODO: s_motor_x_a, s_motor_x_b (shared TIM4), s_motor_z (TIM2) */

void axes_init(void)
{
    motor_init(&s_motor_y, &htim3, TIM_CHANNEL_1,
               Y_DIR_GPIO_Port, Y_DIR_Pin,
               Y_SON_GPIO_Port, Y_SON_Pin);
    motor_enable(&s_motor_y);
}

Motor *axes_get(char letter)
{
    switch (letter) {
        case 'Y': case 'y': return &s_motor_y;
        default:            return NULL;
    }
}
