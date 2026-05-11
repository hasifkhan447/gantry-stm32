#ifndef HOST_IO_H
#define HOST_IO_H

#include <stdint.h>
#include <stdbool.h>

void host_io_init(void);

void host_io_rx_hook(const uint8_t *buf, uint32_t len);

int host_read_byte(uint8_t *out, uint32_t timeout_ms);

bool host_write(const char *s);
bool host_write_buf(const uint8_t *buf, uint16_t len);

#endif
