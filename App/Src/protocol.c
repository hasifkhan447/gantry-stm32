#include "protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "axes.h"
#include "host_io.h"
#include "solenoid.h"

#define LINE_MAX        128
#define DEFAULT_HZ      10000U
#define MAX_HZ          500000U

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
    if (*p == '\0')            { host_write("ERR usage MOVE <axis> <steps> [F <hz>]\r\n"); return; }

    AxisId id = axes_id_from_letter(*p);
    if (id == AXIS_COUNT)      { host_write("ERR unknown axis\r\n"); return; }
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

    if (!axes_submit(id, steps_signed, hz)) {
        host_write("ERR queue full\r\n");
        return;
    }
    host_write("ACK\r\n");
}

static void handle_vac(char *args)
{
    char *p = skip_ws(args);
    if (*p == '\0') {
        host_write(solenoid_get() ? "VAC ON\r\n" : "VAC OFF\r\n");
        return;
    }
    if (tok_is(p, "ON", 2) || tok_is(p, "on", 2)) {
        solenoid_set(true);
    } else if (tok_is(p, "OFF", 3) || tok_is(p, "off", 3)) {
        solenoid_set(false);
    } else if (tok_is(p, "TOGGLE", 6) || tok_is(p, "toggle", 6)
            || tok_is(p, "T", 1)      || tok_is(p, "t", 1)) {
        solenoid_toggle();
    } else {
        host_write("ERR usage VAC [ON|OFF|TOGGLE]\r\n");
        return;
    }
    host_write(solenoid_get() ? "VAC ON\r\n" : "VAC OFF\r\n");
}

static void handle_stat(char *args)
{
    (void)args;
    char buf[48];
    char *w = buf;
    for (AxisId id = 0; id < AXIS_COUNT; ++id) {
        if (w != buf) *w++ = ' ';
        *w++ = axes_letter(id)[0];
        *w++ = ':';
        const char *st = axes_is_busy(id) ? "busy" : "idle";
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
    } else if (tok_is(line, "VAC", 3)) {
        handle_vac(line + 3);
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
