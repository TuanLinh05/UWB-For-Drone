/**
 ******************************************************************************
 * @file    uart_tx.h
 * @brief   Non-blocking UART TX — interrupt-driven ring buffer (Phase 1)
 *
 * Thay thế HAL_UART_Transmit(..., HAL_MAX_DELAY) blocking bằng một ring
 * buffer + ngắt TXE. Vòng ranging chỉ ENQUEUE (không bao giờ đợi UART).
 *
 *   UART_TX_Init()   — gọi 1 lần sau MX_USART1_UART_Init().
 *   UART_TX_Write()  — enqueue N byte; trả về số byte đã nhận (0 nếu drop).
 *
 * Nếu buffer đầy, cả gói bị bỏ và uart_tx_overflow_count++ (giữ nguyên
 * tính toàn vẹn khung, không gửi gói nửa chừng).
 ******************************************************************************
 */

#ifndef UART_TX_H
#define UART_TX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

/** Kích thước ring buffer (phải là luỹ thừa của 2). */
#define UART_TX_BUF_SIZE   1024U

/** Đếm số lần gói bị bỏ do buffer đầy — mục tiêu = 0 trong test dài. */
extern volatile uint32_t uart_tx_overflow_count;

/**
 * @brief  Khởi tạo UART TX non-blocking trên handle đã cấu hình.
 *         Bật NVIC cho USART IRQ (ưu tiên thấp hơn DW1000 EXTI).
 * @param  huart  UART đã init (vd &huart1)
 */
void UART_TX_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Enqueue dữ liệu để gửi qua ngắt (non-blocking).
 * @param  data  con trỏ dữ liệu
 * @param  len   số byte
 * @retval len nếu vừa buffer; 0 nếu bị bỏ (overflow++)
 */
uint32_t UART_TX_Write(const uint8_t *data, uint16_t len);

/**
 * @brief  Số byte đang chờ trong buffer (cho debug/telemetry).
 */
uint16_t UART_TX_Pending(void);

/**
 * @brief  ISR handler — gọi từ USART1_IRQHandler.
 *         (Handler USART1_IRQHandler được định nghĩa sẵn trong uart_tx.c.)
 */
void UART_TX_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_TX_H */
