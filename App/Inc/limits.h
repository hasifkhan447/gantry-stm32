#ifndef LIMITS_H
#define LIMITS_H

#include <stdbool.h>

/* Limit switches and ESTOP.
 *
 * Wiring (see WIRING.txt):
 *   PA1 = X1_LIM  (X- end)         EXTI1
 *   PA2 = X2_LIM  (X+ end)         EXTI2
 *   PA3 = Y1_LIM  (Y- end)         EXTI3
 *   PA4 = Y2_LIM  (Y+ end)         EXTI4
 *   PC13 = ESTOP_IN                EXTI15_10
 *
 * All inputs are pulled up; a closed contact / pressed e-stop pulls the
 * line LOW. EXTI fires on the falling edge.
 *
 * On a limit edge the corresponding axis is aborted immediately (PWM
 * killed inside the ISR). On ESTOP every axis is aborted and the
 * estop_latched flag stays asserted until cleared from the host
 * (RESUME command). */

typedef enum {
    LIM_X1 = 0,
    LIM_X2,
    LIM_Y1,
    LIM_Y2,
    LIM_ESTOP,
    LIM_COUNT
} LimitId;

void limits_init(void);

/* True iff the underlying pin currently reads asserted (active-low). */
bool limits_active(LimitId id);

/* True iff a limit edge has fired since the last clear. */
bool limits_latched(LimitId id);
void limits_clear_latch(LimitId id);
void limits_clear_all(void);

/* The ESTOP latch is sticky: it stays true once raised until cleared. */
bool limits_estop_latched(void);

/* Wake the limits emitter task so it writes a fresh `EVT LIM ...` line
 * to the host. Safe from task context; the emitter coalesces multiple
 * wakeups into a single emission. Call after RESUME or any other
 * operation that mutates latch state outside the EXTI ISR. */
void limits_notify_changed(void);

#endif
