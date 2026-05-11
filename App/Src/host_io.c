#include "host_io.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#include "stm32f3xx_hal.h"
#include "usbd_cdc_if.h"

#define DBG_RX_PIN  GPIO_PIN_10  /* PE10 orange — toggles on every RX byte */
#define DBG_TX_PIN  GPIO_PIN_11  /* PE11 green  — toggles on every TX attempt */

#define HOST_RX_STREAM_BYTES   512
#define HOST_RX_TRIGGER_BYTES  1
#define HOST_TX_RETRY_LIMIT    100   /* ~100 ms total at 1 ms tick */

static StreamBufferHandle_t s_rx_stream;
static StaticStreamBuffer_t s_rx_stream_ctrl;
static uint8_t s_rx_storage[HOST_RX_STREAM_BYTES + 1];

void host_io_init(void)
{
    s_rx_stream = xStreamBufferCreateStatic(
        HOST_RX_STREAM_BYTES,
        HOST_RX_TRIGGER_BYTES,
        s_rx_storage,
        &s_rx_stream_ctrl);
}

void host_io_rx_hook(const uint8_t *buf, uint32_t len)
{
    if (s_rx_stream == NULL || buf == NULL || len == 0) {
        return;
    }
    HAL_GPIO_TogglePin(GPIOE, DBG_RX_PIN);
    BaseType_t hp_task_woken = pdFALSE;
    xStreamBufferSendFromISR(s_rx_stream, buf, len, &hp_task_woken);
    portYIELD_FROM_ISR(hp_task_woken);
}

int host_read_byte(uint8_t *out, uint32_t timeout_ms)
{
    if (s_rx_stream == NULL || out == NULL) {
        return -1;
    }
    TickType_t ticks = (timeout_ms == 0xFFFFFFFFu)
        ? portMAX_DELAY
        : pdMS_TO_TICKS(timeout_ms);
    size_t n = xStreamBufferReceive(s_rx_stream, out, 1, ticks);
    return (n == 1) ? 1 : 0;
}

bool host_write_buf(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0) {
        return false;
    }
    HAL_GPIO_TogglePin(GPIOE, DBG_TX_PIN);
    for (int attempt = 0; attempt < HOST_TX_RETRY_LIMIT; ++attempt) {
        uint8_t r = CDC_Transmit_FS((uint8_t *)buf, len);
        if (r == USBD_OK) {
            return true;
        }
        if (r != USBD_BUSY) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool host_write(const char *s)
{
    if (s == NULL) {
        return false;
    }
    uint16_t len = 0;
    while (s[len] != '\0' && len < 0xFFFF) {
        ++len;
    }
    return host_write_buf((const uint8_t *)s, len);
}
