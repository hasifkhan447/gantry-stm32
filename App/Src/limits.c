#include "limits.h"

#include <stdio.h>

#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "axes.h"
#include "host_io.h"

/* Set to 1 once the physical e-stop is wired to PC13. While this is 0
 * we don't configure GPIOC, don't arm EXTI15_10, and the protocol just
 * reports ES:0/0. A floating PC13 with the internal pull-up could
 * still pick up enough noise to fire EXTI; leaving it unconfigured
 * avoids that entirely. */
#define LIMITS_ESTOP_ENABLED 0

/* Pin map mirrored from WIRING.txt. Keep these adjacent to the table
 * in limits.h. */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    AxisId        axis;       /* axis to abort on edge; AXIS_COUNT = all */
} LimitPin;

#define ESTOP_AXIS_ALL  AXIS_COUNT

static const LimitPin s_pins[LIM_COUNT] = {
    [LIM_X1]    = { GPIOA, GPIO_PIN_1,  AXIS_X         },
    [LIM_X2]    = { GPIOA, GPIO_PIN_2,  AXIS_X         },
    [LIM_Y1]    = { GPIOA, GPIO_PIN_3,  AXIS_Y         },
    [LIM_Y2]    = { GPIOA, GPIO_PIN_4,  AXIS_Y         },
    [LIM_ESTOP] = { GPIOC, GPIO_PIN_13, ESTOP_AXIS_ALL },
};

static volatile bool s_latched[LIM_COUNT];

#define EMIT_STACK_WORDS  192
static StaticTask_t s_emit_tcb;
static StackType_t  s_emit_stack[EMIT_STACK_WORDS];
static TaskHandle_t s_emit_task;

static void limits_emit_task(void *arg);

void limits_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
#if LIMITS_ESTOP_ENABLED
    __HAL_RCC_GPIOC_CLK_ENABLE();
#endif
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_IT_FALLING;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &g);

#if LIMITS_ESTOP_ENABLED
    g.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &g);
#endif

    /* Same numeric priority as the step-timer IRQs (5 = configMAX_-
     * SYSCALL_INTERRUPT_PRIORITY in this build), so neither preempts
     * the other and the abort logic doesn't race with motor_irq_dispatch. */
    const uint32_t prio = 5;
    HAL_NVIC_SetPriority(EXTI1_IRQn,     prio, 0);
    HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, prio, 0);
    HAL_NVIC_SetPriority(EXTI3_IRQn,     prio, 0);
    HAL_NVIC_SetPriority(EXTI4_IRQn,     prio, 0);
#if LIMITS_ESTOP_ENABLED
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, prio, 0);
#endif

    /* Clear EXTI pending bits AND the NVIC pending bits before we
     * enable the IRQs. The pull-up transition during HAL_GPIO_Init
     * briefly looks like a falling edge to EXTI and will both set
     * EXTI->PR and pend the NVIC line; if we don't scrub both, the
     * first IRQ fires immediately with no real switch press. */
    EXTI->PR = (EXTI_PR_PR1 | EXTI_PR_PR2 | EXTI_PR_PR3 | EXTI_PR_PR4
#if LIMITS_ESTOP_ENABLED
              | EXTI_PR_PR10 | EXTI_PR_PR11 | EXTI_PR_PR12 | EXTI_PR_PR13
              | EXTI_PR_PR14 | EXTI_PR_PR15
#endif
              );
    HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI2_TSC_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);
#if LIMITS_ESTOP_ENABLED
    HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
#endif

    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
#if LIMITS_ESTOP_ENABLED
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif

    for (int i = 0; i < LIM_COUNT; ++i) s_latched[i] = false;

    s_emit_task = xTaskCreateStatic(limits_emit_task,
                                    "lim_emit",
                                    EMIT_STACK_WORDS,
                                    NULL,
                                    /* same priority band as protocol task */
                                    3,
                                    s_emit_stack,
                                    &s_emit_tcb);
}

bool limits_active(LimitId id)
{
    if ((unsigned)id >= LIM_COUNT) return false;
#if !LIMITS_ESTOP_ENABLED
    if (id == LIM_ESTOP) return false;   /* GPIOC not configured */
#endif
    const LimitPin *p = &s_pins[id];
    return HAL_GPIO_ReadPin(p->port, p->pin) == GPIO_PIN_RESET;
}

bool limits_latched(LimitId id)
{
    if ((unsigned)id >= LIM_COUNT) return false;
    return s_latched[id];
}

void limits_clear_latch(LimitId id)
{
    if ((unsigned)id >= LIM_COUNT) return;
    s_latched[id] = false;
}

void limits_clear_all(void)
{
    for (int i = 0; i < LIM_COUNT; ++i) s_latched[i] = false;
}

bool limits_estop_latched(void)
{
    return s_latched[LIM_ESTOP];
}

/* ---- event emitter ------------------------------------------------------- */

void limits_notify_changed(void)
{
    if (s_emit_task != NULL) {
        xTaskNotifyGive(s_emit_task);
    }
}

static void limits_notify_from_isr(void)
{
    if (s_emit_task == NULL) return;
    BaseType_t hw = pdFALSE;
    vTaskNotifyGiveFromISR(s_emit_task, &hw);
    portYIELD_FROM_ISR(hw);
}

static void limits_emit_task(void *arg)
{
    (void)arg;
    char line[80];
    static const char *names[LIM_COUNT] = { "X1", "X2", "Y1", "Y2", "ES" };

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Short debounce; collapses bursts (e.g. several limits firing
         * within microseconds of each other) into a single event. */
        vTaskDelay(pdMS_TO_TICKS(8));
        /* Drain any extra wakeups that piled up during the delay. */
        (void)ulTaskNotifyTake(pdTRUE, 0);

        int n = snprintf(line, sizeof(line), "EVT LIM");
        for (int i = 0; i < LIM_COUNT && n < (int)sizeof(line) - 8; ++i) {
            n += snprintf(line + n, sizeof(line) - n,
                          " %s:%d/%d",
                          names[i],
                          limits_active((LimitId)i) ? 1 : 0,
                          s_latched[i] ? 1 : 0);
        }
        if (n < (int)sizeof(line) - 2) {
            line[n++] = '\r';
            line[n++] = '\n';
        }
        host_write_buf((uint8_t *)line, (uint16_t)n);
    }
}

/* ---- ISR dispatch -------------------------------------------------------- */

static void dispatch_pin(uint16_t pin)
{
    for (int i = 0; i < LIM_COUNT; ++i) {
        if (s_pins[i].pin != pin) continue;
        s_latched[i] = true;
        if (s_pins[i].axis == ESTOP_AXIS_ALL) {
            axes_abort_all();
        } else {
            axes_abort(s_pins[i].axis);
        }
        limits_notify_from_isr();
        return;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    dispatch_pin(pin);
}

void EXTI1_IRQHandler(void)     { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);  }
void EXTI3_IRQHandler(void)     { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);  }
void EXTI4_IRQHandler(void)     { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);  }

/* Shared vectors: clear pending on every line they cover, or an
 * unhandled pending bit will keep re-firing the IRQ forever. */
void EXTI2_TSC_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    /* TSC is unused in this project; no further action needed for it. */
}

#if LIMITS_ESTOP_ENABLED
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}
#endif
