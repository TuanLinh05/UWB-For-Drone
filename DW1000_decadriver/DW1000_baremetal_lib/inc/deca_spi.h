/*
 * deca_spi.h
 *
 * Created on: Feb 1, 2026
 * Author: Linh
 * Description: File tiêu đề cho giao tiếp SPI với module DW1000
 */

#ifndef INC_DECA_SPI_H_
#define INC_DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "deca_device_api.h"

// --- Định nghĩa các hàm giao tiếp SPI ---

/*!
 * @brief Ghi dữ liệu vào DW1000 qua SPI
 * @param headerLength: Độ dài header (1-3 bytes tùy lệnh)
 * @param headerBuffer: Buffer chứa lệnh và địa chỉ
 * @param bodylength: Độ dài dữ liệu cần ghi
 * @param bodyBuffer: Buffer chứa dữ liệu cần ghi
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);

/*!
 * @brief Đọc dữ liệu từ DW1000 qua SPI
 * @param headerLength: Độ dài header
 * @param headerBuffer: Buffer chứa lệnh và địa chỉ
 * @param readlength: Độ dài dữ liệu cần đọc
 * @param readBuffer: Buffer để lưu dữ liệu đọc được
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);

// --- Các hàm hỗ trợ hệ thống (System & Mutex) ---

/*!
 * @brief Bắt đầu đoạn code cần bảo vệ (Critical Section)
 * Thường dùng để tắt ngắt EXTI khi đang thao tác biến toàn cục
 */
decaIrqStatus_t decamutexon(void);

/*!
 * @brief Kết thúc đoạn code bảo vệ, bật lại ngắt
 */
void decamutexoff(decaIrqStatus_t s);

/*!
 * @brief Hàm delay mili-giây (Phiên bản chuẩn Decawave)
 */
void deca_sleep(unsigned int time_ms);

/*!
 * @brief Hàm delay mili-giây (Phiên bản tương thích Shinetree)
 * Thêm hàm này để sửa lỗi Warning implicit declaration
 */
void Sleep(unsigned int time_ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_DECA_SPI_H_ */
