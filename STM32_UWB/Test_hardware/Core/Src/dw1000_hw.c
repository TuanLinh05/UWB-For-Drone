#include "dw1000_hw.h"

#define CS_LOW()    HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET)

static void DW1000_ReadReg(uint8_t reg_id, uint8_t *data, uint16_t len)
{
    uint8_t header = reg_id & 0x3F;
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &header, 1, 10);
    HAL_SPI_Receive(&hspi1, data, len, 10);
    CS_HIGH();
}

uint32_t DW1000_ReadDeviceID(void)
{
    uint8_t buf[4] = {0};
    DW1000_ReadReg(DW_REG_DEV_ID, buf, 4);

    return (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
}
