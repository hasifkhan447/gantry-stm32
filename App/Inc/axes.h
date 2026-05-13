#ifndef AXES_H
#define AXES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_COUNT
} AxisId;

void   axes_init(void);

AxisId axes_id_from_letter(char letter);     /* AXIS_COUNT if not 'X'/'Y'/'Z' */
const char *axes_letter(AxisId id);

/* Submit a move. signed_steps < 0 -> reverse direction. Returns false on
 * bad args, unknown axis, or full queue. */
bool   axes_submit(AxisId id, long signed_steps, uint32_t hz);

/* True if the axis is currently stepping OR has queued moves pending. */
bool   axes_is_busy(AxisId id);

#endif
