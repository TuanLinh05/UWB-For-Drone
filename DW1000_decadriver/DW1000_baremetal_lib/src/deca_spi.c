/*
 * deca_spi.c
 *
 * Created on: Feb 1, 2026
 * Author: Linh
 * Description: Triển khai giao tiếp SPI và Delay cho DW1000 sử dụng STM32 HAL
 */

#include "deca_spi.h"
#include "stm32f1xx_hal.h"

// Biến quản lý SPI được khai báo bên main.c
// Đảm bảo bạn đã Init SPI1 trong CubeMX
extern SPI_HandleTypeDef hspi1;

// ---------------------------------------------------------------------------
// 1. Hàm Sleep & Delay
// ---------------------------------------------------------------------------

// Hàm Sleep (Sửa lỗi Warning trong driver Shinetree)
void Sleep(unsigned int time_ms)
{
    HAL_Delay(time_ms);
}

// Hàm deca_sleep (Chuẩn Decawave)
void deca_sleep(unsigned int time_ms)
{
    HAL_Delay(time_ms);
}

// ---------------------------------------------------------------------------
// 2. Hàm Mutex (Quản lý ngắt)
// ---------------------------------------------------------------------------

decaIrqStatus_t decamutexon(void)
{
    // Nếu bạn dùng ngắt ngoài (EXTI) để nhận tín hiệu từ DW1000,
    // bạn nên tắt ngắt ở đây để tránh xung đột dữ liệu.
    // Ví dụ: __disable_irq();
    // Hiện tại để trống vì ta đang dùng Polling hoặc chưa cần bảo vệ quá chặt.
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
    // Bật lại ngắt nếu đã tắt ở trên
    // Ví dụ: __enable_irq();
}

// ---------------------------------------------------------------------------
// 3. Hàm Ghi SPI (Write to SPI)
// ---------------------------------------------------------------------------
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    // Bước 1: Kéo chân CS (Chip Select) xuống thấp để bắt đầu frame
    // Đảm bảo chân 'DW_CS' đã được đặt tên (User Label) trong CubeMX
    HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_RESET);

    // Bước 2: Gửi Header (Lệnh + Địa chỉ thanh ghi)
    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)headerBuffer, headerLength, 100) != HAL_OK)
    {
        // Nếu lỗi, kéo CS lên và thoát
        HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_SET);
        return -1;
    }

    // Bước 3: Gửi Body (Dữ liệu ghi vào thanh ghi) - Nếu có
    if (bodylength > 0)
    {
        if (HAL_SPI_Transmit(&hspi1, (uint8_t *)bodyBuffer, bodylength, 100) != HAL_OK)
        {
            HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_SET);
            return -1;
        }
    }

    // Bước 4: Kéo chân CS lên cao để kết thúc frame
    HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_SET);

    return 0;
}

// ---------------------------------------------------------------------------
// 4. Hàm Đọc SPI (Read from SPI)
// ---------------------------------------------------------------------------
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    // Bước 1: Kéo chân CS xuống thấp
    HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_RESET);

    // Bước 2: Gửi Header để báo cho DW1000 biết muốn đọc thanh ghi nào
    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)headerBuffer, headerLength, 100) != HAL_OK)
    {
        HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_SET);
        return -1;
    }

    // Bước 3: Đọc dữ liệu từ DW1000 trả về
    if (readlength > 0)
    {
        if (HAL_SPI_Receive(&hspi1, readBuffer, readlength, 100) != HAL_OK)
        {
            HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_SET);
            return -1;
        }
    }

    // Bước 4: Kéo chân CS lên cao
    HAL_GPIO_WritePin(DW_CS_GPIO_Port, DW_CS_Pin, GPIO_PIN_SET);

    return 0;
}

// Các hàm giữ chỗ (Placeholder) cho tương thích
int openspi() { return 0; }
int closespi() { return 0; }
