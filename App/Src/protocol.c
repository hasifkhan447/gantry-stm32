#include "protocol.h"

#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "host_io.h"

#define LINE_MAX 128

static void handle_line(char *line, uint16_t len)
{
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ')) {
        line[--len] = '\0';
    }
    if (len == 0) {
        return;
    }

    if (strncmp(line, "PING", 4) == 0 && (line[4] == '\0' || line[4] == ' ')) {
        host_write("PONG\r\n");
    } else if (strncmp(line, "VER", 3) == 0 && (line[3] == '\0' || line[3] == ' ')) {
        host_write("GANTRY 0.1\r\n");
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
