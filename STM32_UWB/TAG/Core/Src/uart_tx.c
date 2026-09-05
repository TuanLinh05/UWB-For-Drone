/**
 ******************************************************************************
 * @file    uart_tx.c
 * @brief   Non-blocking UART TX — interrupt-driven ring buffer (Phase 1)
 *
 * Single-producer (main loop) / single-consumer (USART TXE ISR).
 *   - Producer chỉ ghi s_head.
 *   - Consumer (ISR) chỉ ghi s_tail.
 * Nên không cần khoá; chỉ cần bật TXEIE sau khi đã nạp dữ liệu.
 ******************************************************************************
 */

#include "uart_tx.h"

#define UART_TX_MASK   (UART_TX_BUF_SIZE - 1U)

volatile uint32_t uart_tx_overflow_count = 0;

static UART_HandleTypeDef *s_huart = NULL;
static volatile uint8_t    s_buf[UART_TX_BUF_SIZE];
static volatile uint16_t   s_head = 0;   /* vị trí ghi (producer) */
static volatile uint16_t   s_tail = 0;   /* vị trí đọc (consumer/ISR) */

/* ------------------------------------------------------------------ */
static inline uint16_t buf_count(void)
{
    return (uint16_t)((s_head - s_tail) & UART_TX_MASK);
}

/* Số ô trống; giữ lại 1 ô để phân biệt đầy/rỗng */
static inline uint16_t buf_free(void)
{
    return (uint16_t)(UART_TX_MASK - buf_count());
}

/* ------------------------------------------------------------------ */
void UART_TX_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_head  = 0;
    s_tail  = 0;

    /* Ưu tiên thấp hơn DW1000 EXTI3 (đang là 0). ISR này rất ngắn. */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ------------------------------------------------------------------ */
uint32_t UART_TX_Write(const uint8_t *data, uint16_t len)
{
    if (s_huart == NULL || len == 0)
        return 0;

    /* Giữ nguyên vẹn khung: nếu không đủ chỗ, bỏ cả gói */
    if (len > buf_free())
    {
        uart_tx_overflow_count++;
        return 0;
    }

    uint16_t h = s_head;
    for (uint16_t i = 0; i < len; i++)
    {
        s_buf[h] = data[i];
        h = (uint16_t)((h + 1U) & UART_TX_MASK);
    }
    s_head = h;   /* commit sau khi đã ghi hết dữ liệu */

    /* Kích hoạt truyền: bật ngắt TXE. Nếu DR đang rỗng, ISR chạy ngay. */
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_TXE);

    return len;
}

/* ------------------------------------------------------------------ */
uint16_t UART_TX_Pending(void)
{
    return buf_count();
}

/* ------------------------------------------------------------------ */
void UART_TX_IRQHandler(void)
{
    UART_HandleTypeDef *h = s_huart;
    if (h == NULL)
        return;

    if (__HAL_UART_GET_IT_SOURCE(h, UART_IT_TXE) &&
        __HAL_UART_GET_FLAG(h, UART_FLAG_TXE))
    {
        if (s_tail != s_head)
        {
            /* Ghi DR sẽ tự clear cờ TXE trên STM32F1 */
            h->Instance->DR = (uint16_t)(s_buf[s_tail] & 0xFF);
            s_tail = (uint16_t)((s_tail + 1U) & UART_TX_MASK);
        }

        if (s_tail == s_head)
        {
            /* Hết dữ liệu — tắt ngắt TXE để không bị fire liên tục */
            __HAL_UART_DISABLE_IT(h, UART_IT_TXE);
        }
    }
}

/* ------------------------------------------------------------------ *
 * Vector ngắt USART1. Định nghĩa strong ghi đè weak trong startup.
 * (Nếu sau này bật UART interrupt trong CubeMX, xoá bản sinh trong
 *  stm32f1xx_it.c để tránh trùng ký hiệu.)
 * ------------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    UART_TX_IRQHandler();
}
