#include "axes.h"

#include "main.h"
#include "tim.h"
#include "motor.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define AXIS_STACK_WORDS  256
#define AXIS_QUEUE_DEPTH  4

typedef struct {
    uint32_t steps;
    uint32_t hz;
    bool     positive;
} MoveCmd;

typedef struct {
    Motor               *m0;
    Motor               *m1;          /* NULL except for X dual-drive */
    osMessageQueueId_t   q;
    osThreadId_t         th;
    StaticTask_t         tcb;
    uint32_t             stack[AXIS_STACK_WORDS];
    StaticQueue_t        qcb;
    uint8_t              qbuf[AXIS_QUEUE_DEPTH * sizeof(MoveCmd)];
} Axis;

static Motor s_motor_x_a;
static Motor s_motor_x_b;
static Motor s_motor_y;
static Motor s_motor_z;

static Axis s_axes[AXIS_COUNT];

static void axis_task(void *arg);

static void axis_start(AxisId id, Motor *m0, Motor *m1, const char *name)
{
    Axis *a = &s_axes[id];
    a->m0 = m0;
    a->m1 = m1;

    osMessageQueueAttr_t qattr = {
        .name    = name,
        .cb_mem  = &a->qcb, .cb_size = sizeof(a->qcb),
        .mq_mem  = a->qbuf, .mq_size = sizeof(a->qbuf),
    };
    a->q = osMessageQueueNew(AXIS_QUEUE_DEPTH, sizeof(MoveCmd), &qattr);

    osThreadAttr_t tattr = {
        .name       = name,
        .cb_mem     = &a->tcb,   .cb_size    = sizeof(a->tcb),
        .stack_mem  = a->stack,  .stack_size = sizeof(a->stack),
        .priority   = osPriorityAboveNormal,
    };
    a->th = osThreadNew(axis_task, (void *)(uintptr_t)id, &tattr);

    /* For the X pair both motors share TIM4 and finish on the same IRQ;
     * only the primary registers a notify to keep it to one wakeup. */
    motor_set_notify_task(m0, (void *)a->th);
}

void axes_init(void)
{
    motor_init(&s_motor_x_a, &htim4, TIM_CHANNEL_1,
               X_A_DIR_GPIO_Port, X_A_DIR_Pin,
               X_A_SON_GPIO_Port, X_A_SON_Pin);
    motor_init(&s_motor_x_b, &htim4, TIM_CHANNEL_2,
               X_B_DIR_GPIO_Port, X_B_DIR_Pin,
               X_B_SON_GPIO_Port, X_B_SON_Pin);
    /* X_B is mounted mirrored from X_A, so invert DIR to keep them in lockstep. */
    s_motor_x_b.dir_invert = true;
    motor_init(&s_motor_y,   &htim3, TIM_CHANNEL_1,
               Y_DIR_GPIO_Port,   Y_DIR_Pin,
               Y_SON_GPIO_Port,   Y_SON_Pin);
    motor_init(&s_motor_z,   &htim2, TIM_CHANNEL_1,
               Z_DIR_GPIO_Port,   Z_DIR_Pin,
               Z_SON_GPIO_Port,   Z_SON_Pin);

    motor_enable(&s_motor_x_a);
    motor_enable(&s_motor_x_b);
    motor_enable(&s_motor_y);
    motor_enable(&s_motor_z);

    axis_start(AXIS_X, &s_motor_x_a, &s_motor_x_b, "axisX");
    axis_start(AXIS_Y, &s_motor_y,   NULL,         "axisY");
    axis_start(AXIS_Z, &s_motor_z,   NULL,         "axisZ");
}

AxisId axes_id_from_letter(char letter)
{
    switch (letter) {
        case 'X': case 'x': return AXIS_X;
        case 'Y': case 'y': return AXIS_Y;
        case 'Z': case 'z': return AXIS_Z;
        default:            return AXIS_COUNT;
    }
}

const char *axes_letter(AxisId id)
{
    switch (id) {
        case AXIS_X: return "X";
        case AXIS_Y: return "Y";
        case AXIS_Z: return "Z";
        default:     return "?";
    }
}

bool axes_submit(AxisId id, long signed_steps, uint32_t hz)
{
    if (id >= AXIS_COUNT || signed_steps == 0 || hz == 0) return false;

    MoveCmd c = {
        .steps    = (uint32_t)(signed_steps < 0 ? -signed_steps : signed_steps),
        .hz       = hz,
        .positive = (signed_steps > 0),
    };
    return osMessageQueuePut(s_axes[id].q, &c, 0U, 0U) == osOK;
}

bool axes_is_busy(AxisId id)
{
    if (id >= AXIS_COUNT) return false;
    Axis *a = &s_axes[id];
    if (motor_is_moving(a->m0)) return true;
    if (a->m1 && motor_is_moving(a->m1)) return true;
    if (osMessageQueueGetCount(a->q) > 0) return true;
    return false;
}

static void axis_task(void *arg)
{
    AxisId id = (AxisId)(uintptr_t)arg;
    Axis  *a  = &s_axes[id];

    for (;;) {
        MoveCmd c;
        if (osMessageQueueGet(a->q, &c, NULL, osWaitForever) != osOK) {
            continue;
        }

        motor_set_dir(a->m0, c.positive);
        if (a->m1) motor_set_dir(a->m1, c.positive);

        /* Drop any stale done-notification before kicking off this move. */
        (void)ulTaskNotifyTake(pdTRUE, 0);

        bool ok = a->m1
            ? motor_move_pair(a->m0, a->m1, c.steps, c.hz)
            : motor_move(a->m0, c.steps, c.hz);
        if (!ok) continue;

        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}
