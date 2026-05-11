#include "app_init.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f3xx_hal.h"

#include "main.h"
#include "tim.h"

#include "host_io.h"
#include "motor.h"
#include "protocol.h"

static Motor s_motor_y;

#define PROTOCOL_STACK_WORDS  512

static StaticTask_t s_protocol_tcb;
static uint32_t     s_protocol_stack[PROTOCOL_STACK_WORDS];

static const osThreadAttr_t protocol_task_attr = {
    .name       = "protocol",
    .cb_mem     = &s_protocol_tcb,
    .cb_size    = sizeof(s_protocol_tcb),
    .stack_mem  = s_protocol_stack,
    .stack_size = sizeof(s_protocol_stack),
    .priority   = (osPriority_t) osPriorityAboveNormal,
};

static void debug_leds_init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15, GPIO_PIN_RESET);
}

void app_init(void)
{
    debug_leds_init();
    host_io_init();
    osThreadId_t id = osThreadNew(protocol_task, NULL, &protocol_task_attr);
    if (id == NULL) {
        /* Visible failure: lock up with the red LED stuck on. */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
        for (;;) { }
    }

    motor_init(&s_motor_y, &htim3, TIM_CHANNEL_1,
               Y_DIR_GPIO_Port, Y_DIR_Pin,
               Y_SON_GPIO_Port, Y_SON_Pin);
    motor_set_dir(&s_motor_y, true);
    motor_enable(&s_motor_y);
    osDelay(200);                  /* give the servo time to energize */
    motor_move(&s_motor_y, 200000U, 40000U); /* 200k steps @ 40 kHz = 5 seconds */
}
