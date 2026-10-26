#include "rs485.h"
#include <string.h>

/* ================================================================
 *  Transceiver direction control  —  PA8 (RS485_DE) / PA11 (RS485_RE)
 *
 *  The new 485 chip drives DE and RE from two separate MCU pins:
 *
 *      PA8  (DE) = 1  →  driver enabled, the MCU owns the bus (sending)
 *      PA8  (DE) = 0  →  driver high-Z, bus released        (idle)
 *      PA11 (RE) = 1  →  receiver enabled                   (receiving)
 *      PA11 (RE) = 0  →  receiver disabled while transmitting, so the
 *                        device never sees its own echo
 *
 *  Idle state is DE low + RE high, i.e. the bus is released and the
 *  receiver is always listening.  Every transmission on USART1 has to
 *  go through rs485_transmit()/rs485_tx() so the two pins are switched
 *  around the transfer.
 *
 *  Pin macros live in rs485.h (RS485_DE_ENABLE/DISABLE, RS485_RE_...).
 * ================================================================ */

/* ================================================================
 *  Internal state
 * ================================================================ */
static UART_HandleTypeDef *g_rs485_huart;      /* USART1                */
static rs485_frame_cb      g_rs485_frame_cb;   /* frame handler          */
static volatile uint8_t    g_rs485_rx_dma[RS485_RX_BUF_SIZE];
static ringbuf_t           g_rs485_ring;
static volatile uint8_t    g_rs485_ring_buf[RS485_RX_BUF_SIZE * 2]; /* larger to absorb bursts */
static volatile uint8_t    g_rs485_rx_running;  /* DMA started flag    */

/* Give the line driver a moment to take over the bus before the first
   start bit goes out.  ~1 us is plenty (one bit at 460800 bps = 2.17 us),
   and it costs nothing next to a 29-byte frame. */
static inline void rs485_bus_settle(void)
{
    for (volatile uint32_t i = 0; i < 24U; i++) { __NOP(); }
}

/* ---- TX helpers ---- */

static void rs485_tx(const uint8_t *data, uint16_t len)
{
    rs485_transmit(data, len, 100);
}

/* ================================================================
 *  rs485_init
 * ================================================================ */
void rs485_init(UART_HandleTypeDef *huart, rs485_frame_cb frame_cb)
{
    g_rs485_huart     = huart;
    g_rs485_frame_cb  = frame_cb;
    g_rs485_rx_running = 0;

    ringbuf_init(&g_rs485_ring, g_rs485_ring_buf, sizeof(g_rs485_ring_buf));

    /* Switch transceiver to RX mode: driver off (PA8 low), receiver on
       (PA11 high).  From now on the direction pins are driven by the
       send/receive helpers below. */
    RS485_DE_DISABLE();
    RS485_RE_ENABLE();

    /* Enable IDLE line detection interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    /* Start DMA reception */
    rs485_rx_restart();
}

/* ================================================================
 *  rs485_rx_harvest  —  move everything the DMA captured so far into
 *  the ring buffer.  Called before pausing reception for a transmit,
 *  so a frame that has just ended is never thrown away.
 *
 *  Valid while the DMA has not wrapped yet — reception is restarted at
 *  every IDLE gap, so this always holds for one bus frame.
 * ================================================================ */
static void rs485_rx_harvest(void)
{
    if (!g_rs485_rx_running || g_rs485_huart->hdmarx == NULL) {
        return;
    }

    uint16_t total = (uint16_t)sizeof(g_rs485_rx_dma);
    uint16_t rx_cnt = total - (uint16_t)__HAL_DMA_GET_COUNTER(g_rs485_huart->hdmarx);

    if (rx_cnt && rx_cnt <= total) {
        ringbuf_write(&g_rs485_ring, (const uint8_t *)g_rs485_rx_dma, rx_cnt);
    }
}

/* ================================================================
 *  rs485_rx_pause  —  stop DMA capture without touching the pins
 * ================================================================ */
static void rs485_rx_pause(void)
{
    /* Clear the flag first: the IDLE ISR must not push bytes into the
       ring buffer nor restart the DMA while we are switching to TX. */
    g_rs485_rx_running = 0;

    if (g_rs485_huart->hdmarx) {
        HAL_UART_DMAStop(g_rs485_huart);
    }
}

/* ================================================================
 *  rs485_rx_restart
 * ================================================================ */
void rs485_rx_restart(void)
{
    /* Make sure DMA is not running */
    rs485_rx_pause();

    HAL_UART_Receive_DMA(g_rs485_huart,
                         (uint8_t *)g_rs485_rx_dma,
                         sizeof(g_rs485_rx_dma));
    g_rs485_rx_running = 1;
}

/* ================================================================
 *  rs485_enter_tx  —  call BEFORE putting bytes on the bus
 *
 *  1. wait until the previous frame has been fully shifted out,
 *  2. harvest then stop DMA reception (no echo garbage in the ring),
 *  3. PA11 = 0 → disable the receiver,
 *  4. PA8  = 1 → enable the line driver.
 * ================================================================ */
void rs485_enter_tx(void)
{
    /* Wait for any previous TX to finish */
    while (__HAL_UART_GET_FLAG(g_rs485_huart, UART_FLAG_TC) == RESET) {}

    rs485_rx_harvest();          /* keep a frame that just finished       */
    rs485_rx_pause();            /* stop capture while we drive the bus    */
    RS485_RE_DISABLE();          /* PA11 = 0 — do not hear our own echo    */
    RS485_DE_ENABLE();           /* PA8  = 1 — drive the bus               */
    rs485_bus_settle();          /* let the driver take over before bit 0  */
}

/* ================================================================
 *  rs485_enter_rx  —  call AFTER the last byte left the shift register
 *
 *  1. wait for TC so the final stop bit is really on the wire,
 *  2. PA8  = 0 → release the bus (idle),
 *  3. PA11 = 1 → enable the receiver,
 *  4. restart DMA reception from a clean state.
 * ================================================================ */
void rs485_enter_rx(void)
{
    /* Wait until the last byte has completely shifted out */
    while (__HAL_UART_GET_FLAG(g_rs485_huart, UART_FLAG_TC) == RESET) {}

    RS485_DE_DISABLE();          /* PA8  = 0 — release the bus, idle level     */
    RS485_RE_ENABLE();           /* PA11 = 1 — listening again                 */
    rs485_rx_restart();          /* fresh DMA reception                        */
}

/* ================================================================
 *  rs485_transmit  —  one-shot: TX mode → send → RX mode
 *
 *  Every byte that goes onto the 485 bus through USART1 (data frames,
 *  ACKs and debug/log strings alike) must be sent with this helper.
 * ================================================================ */
void rs485_transmit(const uint8_t *data, uint16_t len, uint32_t timeout)
{
    if (g_rs485_huart == NULL || data == NULL || len == 0U) {
        return;
    }

    rs485_enter_tx();
    HAL_UART_Transmit(g_rs485_huart, (uint8_t *)data, len, timeout);
    rs485_enter_rx();
}

/* ================================================================
 *  rs485_rx_idle_cb  — called from the USART1 IRQ handler
 * ================================================================ */
void rs485_rx_idle_cb(uint16_t dma_remaining)
{
    uint16_t total   = sizeof(g_rs485_rx_dma);
    uint16_t rx_cnt  = total - dma_remaining;

    /* Reception is paused while we transmit — ignore IDLE in that window
       (the ISR must not restart a DMA that TX deliberately stopped). */
    if (!g_rs485_rx_running) {
        return;
    }

    if (rx_cnt && rx_cnt <= total) {
        ringbuf_write(&g_rs485_ring, (const uint8_t *)g_rs485_rx_dma, rx_cnt);
    }

    /* Restart DMA */
    rs485_rx_restart();
}

/* ================================================================
 *  rs485_process  —  single-frame scanner (call from main loop)
 *
 *  Walks the ring buffer byte-by-byte, looking for an AA 55 header
 *  followed by a 0D 0A tail.  When a complete frame is found, it
 *  dispatches through the callback and advances past the tail.
 *
 *  CMD byte interpretation:
 *    0x00 → continuous data frame:    payload is raw 6x float32 + seq
 *    other → command frame:           payload after CMD
 * ================================================================ */
void rs485_process(void)
{
    while (ringbuf_avail(&g_rs485_ring) >= 4) {   /* minimum: SOF + EOF */

        /* ---- Step 1: Align to AA 55 ---- */
        uint8_t b0 = ringbuf_peek(&g_rs485_ring, 0);
        if (b0 != RS485_FRAME_SOF_HI) {
            ringbuf_get(&g_rs485_ring);             /* skip 1 byte */
            continue;
        }
        uint8_t b1 = ringbuf_peek(&g_rs485_ring, 1);
        if (b1 == RS485_FRAME_SOF_LO) break;        /* found SOF */
        ringbuf_get(&g_rs485_ring);                 /* skip AA, retry */
    }

    if (ringbuf_avail(&g_rs485_ring) < 4) return;

    /* ---- Step 2: Search for 0D 0A EOF within available data ---- */
    uint16_t avail = ringbuf_avail(&g_rs485_ring);
    int16_t  eof_pos = -1;
    for (uint16_t i = 3; (i + 1) < avail; i++) {
        if (ringbuf_peek(&g_rs485_ring, i)     == RS485_FRAME_EOF_HI &&
            ringbuf_peek(&g_rs485_ring, i + 1) == RS485_FRAME_EOF_LO) {
            eof_pos = (int16_t)i;
            break;
        }
    }
    if (eof_pos < 3) return;   /* no complete frame yet */

    /* ---- Step 3: Extract frame contents ---- */
    uint16_t frame_len = (uint16_t)(eof_pos + 2);  /* including EOF */
    uint8_t  cmd;
    uint16_t payload_len;

    if (frame_len == 4) {
        /* AA 55 0D 0A — empty frame, discard */
        cmd = 0xFF;
        payload_len = 0;
    } else {
        cmd         = ringbuf_peek(&g_rs485_ring, 2);
        payload_len = frame_len - 5;               /* -SOF(2) -CMD(1) -EOF(2) */
    }

    /* Read payload into a temp buffer */
    uint8_t payload[RS485_MAX_PAYLOAD];
    /* Skip AA + 55 */
    ringbuf_skip(&g_rs485_ring, 2);
    /* Read CMD */
    if (cmd != 0xFF) ringbuf_get(&g_rs485_ring);
    /* Read payload */
    if (payload_len && payload_len <= RS485_MAX_PAYLOAD) {
        ringbuf_read(&g_rs485_ring, payload, payload_len);
    }
    /* Skip EOF 0D 0A */
    uint8_t e1 = ringbuf_get(&g_rs485_ring);
    uint8_t e2 = ringbuf_get(&g_rs485_ring);
    (void)e1; (void)e2;

    /* ---- Step 4: Dispatch ---- */
    if (cmd != 0xFF && g_rs485_frame_cb) {
        g_rs485_frame_cb(cmd, payload, payload_len);
    }
}

/* ================================================================
 *  rs485_send_frame  —  send AA 55 [CMD] [PAYLOAD...] 0D 0A
 * ================================================================ */
void rs485_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t tx[RS485_TX_BUF_SIZE];
    uint16_t pos = 0;

    tx[pos++] = RS485_FRAME_SOF_HI;
    tx[pos++] = RS485_FRAME_SOF_LO;
    tx[pos++] = cmd;
    if (payload && len && len <= (RS485_TX_BUF_SIZE - 5)) {
        memcpy(tx + pos, payload, len);
        pos += len;
    }
    tx[pos++] = RS485_FRAME_EOF_HI;
    tx[pos++] = RS485_FRAME_EOF_LO;

    rs485_tx(tx, pos);
}

/* ================================================================
 *  rs485_send_raw  —  send AA 55 [DATA...] 0D 0A  (no cmd byte)
 * ================================================================ */
void rs485_send_raw(const uint8_t *data, uint16_t len)
{
    uint8_t tx[RS485_TX_BUF_SIZE];
    uint16_t pos = 0;

    tx[pos++] = RS485_FRAME_SOF_HI;
    tx[pos++] = RS485_FRAME_SOF_LO;
    if (data && len && len <= (RS485_TX_BUF_SIZE - 4)) {
        memcpy(tx + pos, data, len);
        pos += len;
    }
    tx[pos++] = RS485_FRAME_EOF_HI;
    tx[pos++] = RS485_FRAME_EOF_LO;

    rs485_tx(tx, pos);
}

void original_send(const uint8_t *payload, uint16_t len)
{
    rs485_tx(payload, len);
}

void just_float_send_raw(const uint8_t *data, uint16_t len)
{
    uint8_t tx[RS485_TX_BUF_SIZE];
    uint16_t pos = 0;

    if (data && len && len <= (RS485_TX_BUF_SIZE - 4)) {
        memcpy(tx + pos, data, len);
        pos += len;
    }

    rs485_tx(tx, pos);
}
