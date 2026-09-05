#ifndef DW1000_HW_H
#define DW1000_HW_H

#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi1;

/* Pin definitions */
#define SPI_CS_PORT         GPIOA
#define SPI_CS_PIN          GPIO_PIN_4

#define DW_RSTN_PORT        GPIOA
#define DW_RSTN_PIN         GPIO_PIN_2

#define LED_PORT            GPIOC
#define LED_PIN             GPIO_PIN_13

/* Register Map */
#define DW_REG_DEV_ID       0x00

uint32_t DW1000_ReadDeviceID(void);

#endif /* DW1000_HW_H */
