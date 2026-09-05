#ifndef _DECA_TYPES_H_
#define _DECA_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

// --- Định nghĩa các kiểu dữ liệu cũ cho tương thích ---
#ifndef uint8
typedef uint8_t  uint8;
#endif

#ifndef uint16
typedef uint16_t uint16;
#endif

#ifndef uint32
typedef uint32_t uint32;
#endif

#ifndef int8
typedef int8_t   int8;
#endif

#ifndef int16
typedef int16_t  int16;
#endif

#ifndef int32
typedef int32_t  int32;
#endif

#ifndef uint64
typedef uint64_t uint64;
#endif

#ifndef int64
typedef int64_t  int64;
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

typedef int boolean;

// --- THÊM DÒNG NÀY ĐỂ SỬA LỖI SLEEP ---
// Khai báo hàm Sleep để driver DW1000 gọi được
void Sleep(unsigned int time_ms);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_TYPES_H_ */
