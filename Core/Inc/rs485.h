#ifndef __RS485_H__
#define __RS485_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ringbuf.h"
#include <stdint.h>

/* ================================================================
 *  RS485 Frame I/O  — USART2, 460800 bps, 8N1
 *
 *  Frame format:  AA 55 [CMD] [PAYLOAD...] 0D 0A
 *  Reception uses DMA + UART IDLE line detection.
 *
 *  Transceiver direction pins (new 485 chip):
 *      PA8  RS485_DE : 1 = driver on (sending),  0 = idle
 *      PA11 RS485_RE : 1 = receiver on (receiving), 0 = off
 *  Idle state = DE low + RE high.  Both pins are switched
 *  automatically around every transmission (rs485_transmit).
 *  ================================================================ */

#define RS485_FRAME_SOF     0xAA55   /* Start-of-Frame (little-endian) */
#define RS485_FRAME_SOF_HI   0xAA
#define RS485_FRAME_SOF_LO   0x55
#define RS485_FRAME_EOF_HI   0x0D
#define RS485_FRAME_EOF_LO   0x0A

/* ---- Transceiver direction control ---- */
#define RS485_DE_ENABLE()   HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define RS485_DE_DISABLE()  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)
#define RS485_RE_DISABLE()   HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET)
#define RS485_RE_ENABLE()  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET)

#define RS485_RX_BUF_SIZE   512
#define RS485_TX_BUF_SIZE   256
#define RS485_MAX_PAYLOAD   250

/** One decoded frame (callback-driven frame handler) */
typedef void (*rs485_frame_cb)(uint8_t cmd, const uint8_t *payload,
                                uint16_t len);

/**
 * Initialise RS485 subsystem.
 * @param huart   pointer to USART2 handle (CubeMX)
 * @param frame_cb callback for each fully-received frame (can be NULL)
 *
 * Starts DMA reception with IDLE-line detection.
 */
void rs485_init(UART_HandleTypeDef *huart, rs485_frame_cb frame_cb);

/**
 * Periodic frame scanner — call from main loop.
 * Extracts complete AA55...0D0A frames from the ring buffer
 * and dispatches them through the registered callback.
 */
void rs485_process(void);

/**
 * Restart DMA reception (call after error or first init).
 * Blocks until the DMA stream is restarted.
 */
void rs485_rx_restart(void);

/**
 * Switch the transceiver to transmit mode — call BEFORE sending.
 * Waits for the previous frame to leave the shift register, stops DMA
 * reception, then PA11 (RE) = 0 and PA8 (DE) = 1.
 */
void rs485_enter_tx(void);

/**
 * Switch the transceiver back to receive mode — call AFTER sending.
 * Waits for TC (last stop bit on the wire), then PA8 (DE) = 0,
 * PA11 (RE) = 1 and restarts DMA reception.
 */
void rs485_enter_rx(void);

/**
 * Blocking one-shot transmit on the 485 bus with correct direction
 * control: enter TX mode → HAL_UART_Transmit → enter RX mode.
 *
 * Use this for EVERY byte sent on USART1 (data frames, ACKs, debug
 * strings).  Transmission is skipped when huart/data is NULL or
 * len == 0.
 *
 * @param data    bytes to send
 * @param len     number of bytes
 * @param timeout HAL timeout in ms
 */
void rs485_transmit(const uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * Called from the USART1 IRQ handler when IDLE flag is set.
 * Transfers DMA-received bytes into the ring buffer and restarts DMA.
 * Returns immediately while reception is paused for a transmission.
 * @param dma_remaining  hdma->Instance->CNDTR value at the time of IRQ
 */
void rs485_rx_idle_cb(uint16_t dma_remaining);

/**
 * Send a complete RS485 frame (blocking).
 * Builds:  AA 55 [cmd] [payload...] 0D 0A
 * Waits for TX completion.
 */
void rs485_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len);

/**
 * Send a frame with the generic format AA 55 [data...] 0D 0A
 * (no extra command byte).
 */
void rs485_send_raw(const uint8_t *data, uint16_t len);

void original_send(const uint8_t *payload, uint16_t len);

/**
 * Send a frame with the format [data...] 00 00 80 7F
 * (no extra command byte).
 */
void just_float_send_raw(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H__ */
