#include "protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "axes.h"
#include "host_io.h"
#include "motor.h"

#define LINE_MAX        128
#define DEFAULT_HZ      10000U
#define MAX_HZ          200000U

static char *skip_ws(char *p)
{
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

static bool tok_is(const char *line, const char *kw, uint16_t kwlen)
{
    return strncmp(line, kw, kwlen) == 0
        && (line[kwlen] == '\0' || line[kwlen] == ' ' || line[kwlen] == '\t');
}

static void handle_move(char *args)
{
    char *p = skip_ws(args);

    char axis_letter = *p;
    if (axis_letter == '\0') { host_write("ERR usage MOVE <axis> <steps> [F <hz>]\r\n"); return; }
    Motor *m = axes_get(axis_letter);
    if (m == NULL)             { host_write("ERR unknown axis\r\n"); return; }
    p++;
    p = skip_ws(p);

    char *end = NULL;
    long steps_signed = strtol(p, &end, 10);
    if (end == p)              { host_write("ERR missing steps\r\n"); return; }
    p = skip_ws(end);

    uint32_t hz = DEFAULT_HZ;
    if (*p == 'F' || *p == 'f') {
        p = skip_ws(p + 1);
        long hz_signed = strtol(p, &end, 10);
        if (end == p || hz_signed <= 0) { host_write("ERR bad F\r\n"); return; }
        if ((uint32_t)hz_signed > MAX_HZ) hz_signed = MAX_HZ;
        hz = (uint32_t)hz_signed;
    }

    if (steps_signed == 0)     { host_write("ERR zero steps\r\n"); return; }
    if (motor_is_moving(m))    { host_write("ERR busy\r\n"); return; }

    bool positive = (steps_signed > 0);
    uint32_t abs_steps = positive ? (uint32_t)steps_signed
                                   : (uint32_t)(-steps_signed);
    motor_set_dir(m, positive);
    if (!motor_move(m, abs_steps, hz)) {
        host_write("ERR move failed\r\n");
        return;
    }
    host_write("ACK\r\n");
}

static void handle_stat(char *args)
{
    (void)args;
    char buf[48];
    const char axes[] = "Y";   /* extend to "XYZ" as we wire them */
    char *w = buf;
    for (uint8_t i = 0; axes[i] != '\0'; ++i) {
        Motor *m = axes_get(axes[i]);
        if (m == NULL) continue;
        if (w != buf) *w++ = ' ';
        *w++ = axes[i];
        *w++ = ':';
        const char *st = motor_is_moving(m) ? "busy" : "idle";
        while (*st) *w++ = *st++;
    }
    *w++ = '\r';
    *w++ = '\n';
    host_write_buf((uint8_t *)buf, (uint16_t)(w - buf));
}

static void handle_line(char *line, uint16_t len)
{
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) {
        line[--len] = '\0';
    }
    if (len == 0) {
        return;
    }

    if (tok_is(line, "PING", 4)) {
        host_write("PONG\r\n");
    } else if (tok_is(line, "VER", 3)) {
        host_write("GANTRY 0.1\r\n");
    } else if (tok_is(line, "MOVE", 4)) {
        handle_move(line + 4);
    } else if (tok_is(line, "STAT", 4)) {
        handle_stat(line + 4);
    } else {
        host_write("ERR unknown\r\n");
    }
}

void protocol_task(void *arg)
{
    (void)arg;
    static char line[LINE_MAX];
    uint16_t cursor = 0;
    bool overflow = false;

    for (;;) {
        uint8_t b;
        if (host_read_byte(&b, 0xFFFFFFFFu) != 1) {
            continue;
        }

        if (b == '\n' || b == '\r') {
            if (cursor == 0 && !overflow) {
                continue;
            }
            if (overflow) {
                host_write("ERR overflow\r\n");
            } else {
                line[cursor] = '\0';
                handle_line(line, cursor);
            }
            cursor = 0;
            overflow = false;
            continue;
        }

        if (cursor + 1 >= LINE_MAX) {
            overflow = true;
            continue;
        }
        line[cursor++] = (char)b;
    }
}
