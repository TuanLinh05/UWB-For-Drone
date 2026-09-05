/**
  ******************************************************************************
  * @file    dw1000_hw.c
  * @brief   DW1000 UWB Hardware Driver — Implementation
  *
  * Hardware SPI: safe initialization rate, then verified fast runtime rate.
  *
  * Cấu hình PHY THỰC TẾ: Channel 5, PRF 16MHz, Preamble 256, PAC 16,
  * Preamble Code 4, 6.8 Mbps, SFD standard (phải KHỚP với TAG để liên lạc)
  ******************************************************************************
  */

#include "dw1000_hw.h"
#include <string.h>

/* ========================================================================== */
/*                     GPIO MACROS (same as test project)                       */
/* ========================================================================== */

#define CS_LOW()    HAL_GPIO_WritePin(SPI_CS_PORT,   SPI_CS_PIN,   GPIO_PIN_RESET)
#define CS_HIGH()   HAL_GPIO_WritePin(SPI_CS_PORT,   SPI_CS_PIN,   GPIO_PIN_SET)
#define SCK_LOW()   HAL_GPIO_WritePin(SPI_SCK_PORT,  SPI_SCK_PIN,  GPIO_PIN_RESET)
#define SCK_HIGH()  HAL_GPIO_WritePin(SPI_SCK_PORT,  SPI_SCK_PIN,  GPIO_PIN_SET)
#define MOSI_LOW()  HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_RESET)
#define MOSI_HIGH() HAL_GPIO_WritePin(SPI_MOSI_PORT, SPI_MOSI_PIN, GPIO_PIN_SET)
#define MISO_READ() HAL_GPIO_ReadPin(SPI_MISO_PORT,  SPI_MISO_PIN)

volatile uint8_t dw1000_spi_mhz = DW_SPI_INIT_MHZ;

#define LED_ON()    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)   /* Active LOW */
#define LED_OFF()   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_TOGGLE() HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

/* ========================================================================== */
/*                     SPI HARDWARE TRANSFER                                   */
/* ========================================================================== */

static uint8_t SPI_TransferByte(uint8_t tx_byte)
{
    uint8_t rx_byte = 0;
    HAL_SPI_TransmitReceive(&hspi1, &tx_byte, &rx_byte, 1, 10);
    return rx_byte;
}

static void SPI_TransferBytes(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (tx != NULL && rx != NULL) {
        HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 100);
    } else if (tx != NULL) {
        HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, 100);
    } else if (rx != NULL) {
        HAL_SPI_Receive(&hspi1, rx, len, 100);
    }
}

static uint8_t SPI_ApplyRuntimeRate(uint32_t prescaler, uint8_t rate_mhz)
{
    CS_HIGH();
    hspi1.Init.BaudRatePrescaler = prescaler;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        return 0U;

    dw1000_spi_mhz = rate_mhz;
    return (DW1000_ReadDeviceID() == DW1000_DEVICE_ID) ? 1U : 0U;
}

uint8_t DW1000_EnableFastSPI(void)
{
    if (SPI_ApplyRuntimeRate(SPI_BAUDRATEPRESCALER_4, 16U))
        return 16U;
    if (SPI_ApplyRuntimeRate(SPI_BAUDRATEPRESCALER_8, 8U))
        return 8U;
    if (SPI_ApplyRuntimeRate(SPI_BAUDRATEPRESCALER_32, 2U))
        return 2U;

    dw1000_spi_mhz = 0U;
    return 0U;
}

/* ========================================================================== */
/*                     GPIO INITIALIZATION                                     */
/* ========================================================================== */

/**
 * @brief Initialize all GPIO pins for DW1000 communication.
 *
 * IMPORTANT: SPI pins use GPIO_SPEED_FREQ_LOW to prevent signal ringing
 * on the breadboard/PCB traces. This is critical for reliable SPI at
 * our bit-bang speeds (~33 kHz effective clock).
 */
static void DW1000_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* --- SPI Output Pin: CS (Push-Pull, HIGH speed) --- */
    GPIO_InitStruct.Pin   = SPI_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* --- DW1000 RSTN: Start as input (high-Z), will be driven when resetting --- */
    GPIO_InitStruct.Pin  = DW_RSTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DW_RSTN_PORT, &GPIO_InitStruct);

    /* --- DW1000 IRQ: Input (active high) --- */
    GPIO_InitStruct.Pin  = DW_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DW_IRQ_PORT, &GPIO_InitStruct);

    /* --- EXTON: Input --- */
    GPIO_InitStruct.Pin  = DW_EXTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DW_EXTON_PORT, &GPIO_InitStruct);

    /* --- WAKE: Input --- */
    GPIO_InitStruct.Pin  = DW_WAKE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(DW_WAKE_PORT, &GPIO_InitStruct);

    /* --- LED: Push-Pull Output (start OFF = HIGH for active-low) --- */
    GPIO_InitStruct.Pin   = LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    /* Set initial pin states */
    CS_HIGH();      /* CS idle high (deselected) */
    SCK_LOW();      /* SCK idle low (CPOL = 0) */
    MOSI_LOW();     /* MOSI idle low */
    LED_OFF();      /* LED off */
}

/* ========================================================================== */
/*                     DW1000 RESET                                            */
/* ========================================================================== */

/**
 * @brief Hardware reset of the DW1000.
 *
 * Drives RSTN low via open-drain output, then releases it as a floating
 * input (the DW1000 has an internal pull-up on RSTN).
 */
static void DW1000_Reset(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Drive RSTN low using open-drain mode */
    GPIO_InitStruct.Pin   = DW_RSTN_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DW_RSTN_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DW_RSTN_PORT, DW_RSTN_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);  /* Hold reset for 2 ms */

    /* Release RSTN — set back to input (high-Z), internal pull-up brings it high */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DW_RSTN_PORT, &GPIO_InitStruct);

    HAL_Delay(50);  /* Wait for DW1000 to come out of reset and stabilize */
}

/* ========================================================================== */
/*                     BASIC REGISTER READ / WRITE (1-byte header)             */
/* ========================================================================== */

/**
 * @brief Read a DW1000 register using a 1-byte SPI header.
 * @param reg_id   Register file ID (0x00–0x3F)
 * @param data     Buffer to store read data
 * @param len      Number of bytes to read
 *
 * Header byte: bit7=0(read), bit6=0(no sub-index), bits5:0=reg_id
 */
static void DW1000_ReadReg(uint8_t reg_id, uint8_t *data, uint16_t len)
{
    uint8_t header = reg_id & 0x3F;  /* Read: bit7=0, SubIdx=0: bit6=0 */

    CS_LOW();
    SPI_TransferByte(header);
    SPI_TransferBytes(NULL, data, len);
    CS_HIGH();
}

/**
 * @brief Write a DW1000 register using a 1-byte SPI header.
 * @param reg_id   Register file ID (0x00–0x3F)
 * @param data     Data bytes to write
 * @param len      Number of bytes to write
 *
 * Header byte: bit7=1(write), bit6=0(no sub-index), bits5:0=reg_id
 */
static void DW1000_WriteReg(uint8_t reg_id, const uint8_t *data, uint16_t len)
{
    uint8_t header = 0x80 | (reg_id & 0x3F);  /* Write: bit7=1, SubIdx=0: bit6=0 */

    CS_LOW();
    SPI_TransferByte(header);
    SPI_TransferBytes(data, NULL, len);
    CS_HIGH();
}

/* ========================================================================== */
/*                     SUB-REGISTER READ / WRITE (2-3 byte header)             */
/* ========================================================================== */

/**
 * @brief Read a DW1000 sub-register.
 * @param reg_id    Register file ID (0x00–0x3F)
 * @param sub_addr  Sub-register address (0–0x7FFF)
 * @param data      Buffer to store read data
 * @param len       Number of bytes to read
 *
 * Header format:
 *   Byte 1: R/W(bit7)=0, SubIdx(bit6)=1, RegID(bits5:0)
 *   Byte 2: Ext(bit7), SubAddr[6:0]
 *   Byte 3 (if Ext=1): SubAddr[14:7]
 */
static void DW1000_ReadSubReg(uint8_t reg_id, uint16_t sub_addr, uint8_t *data, uint16_t len)
{
    uint8_t header1 = 0x40 | (reg_id & 0x3F);  /* Read + SubIdx=1 */

    CS_LOW();
    SPI_TransferByte(header1);

    if (sub_addr > 0x7F)
    {
        /* Extended addressing: 3-byte header */
        uint8_t header2 = 0x80 | (sub_addr & 0x7F);    /* Ext=1, SubAddr[6:0] */
        uint8_t header3 = (sub_addr >> 7) & 0xFF;       /* SubAddr[14:7] */
        SPI_TransferByte(header2);
        SPI_TransferByte(header3);
    }
    else
    {
        /* Short addressing: 2-byte header */
        uint8_t header2 = sub_addr & 0x7F;             /* Ext=0, SubAddr[6:0] */
        SPI_TransferByte(header2);
    }

    SPI_TransferBytes(NULL, data, len);
    CS_HIGH();
}

/**
 * @brief Write a DW1000 sub-register.
 * @param reg_id    Register file ID (0x00–0x3F)
 * @param sub_addr  Sub-register address (0–0x7FFF)
 * @param data      Data bytes to write
 * @param len       Number of bytes to write
 *
 * Header format:
 *   Byte 1: R/W(bit7)=1, SubIdx(bit6)=1, RegID(bits5:0)
 *   Byte 2: Ext(bit7), SubAddr[6:0]
 *   Byte 3 (if Ext=1): SubAddr[14:7]
 */
static void DW1000_WriteSubReg(uint8_t reg_id, uint16_t sub_addr, const uint8_t *data, uint16_t len)
{
    uint8_t header1 = 0xC0 | (reg_id & 0x3F);  /* Write + SubIdx=1 */

    CS_LOW();
    SPI_TransferByte(header1);

    if (sub_addr > 0x7F)
    {
        /* Extended addressing: 3-byte header */
        uint8_t header2 = 0x80 | (sub_addr & 0x7F);    /* Ext=1, SubAddr[6:0] */
        uint8_t header3 = (sub_addr >> 7) & 0xFF;       /* SubAddr[14:7] */
        SPI_TransferByte(header2);
        SPI_TransferByte(header3);
    }
    else
    {
        /* Short addressing: 2-byte header */
        uint8_t header2 = sub_addr & 0x7F;             /* Ext=0, SubAddr[6:0] */
        SPI_TransferByte(header2);
    }

    SPI_TransferBytes(data, NULL, len);
    CS_HIGH();
}

/* ========================================================================== */
/*                     DEVICE ID                                               */
/* ========================================================================== */

/**
 * @brief Read the DW1000 Device ID (register 0x00, 4 bytes).
 * @retval 32-bit device ID — should be 0xDECA0130 for DW1000
 */
uint32_t DW1000_ReadDeviceID(void)
{
    uint8_t buf[4];
    DW1000_ReadReg(DW_REG_DEV_ID, buf, 4);

    /* DW1000 returns bytes in little-endian order */
    return (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
}

/* ========================================================================== */
/*                     INITIALIZATION                                          */
/* ========================================================================== */

/**
 * @brief Full DW1000 initialization sequence.
 *
 * Steps:
 *   1. Initialize GPIO pins
 *   2. Hardware reset
 *   3. Verify Device ID (must read 0xDECA0130)
 *   4. Load LDE microcode from OTP memory
 *
 * @retval 0 = success, -1 = device ID mismatch
 */
int DW1000_Init(void)
{
    /* Step 1: Initialize all GPIO pins */
    DW1000_GPIO_Init();

    /* Step 2: Hardware reset */
    DW1000_Reset();

    /* Step 3: Verify Device ID */
    uint32_t dev_id = DW1000_ReadDeviceID();
    if (dev_id != DW1000_DEVICE_ID)
    {
        return -1;  /* Wrong device or SPI communication failure */
    }

    /* Step 4: Load LDE microcode from OTP memory.
     * Sequence from DW1000 User Manual section 2.5.5.10:
     *   a) Enable LDE clock: write 0x0301 to PMSC_CTRL0 (0x36:00)
     *   b) Trigger OTP load:  write 0x8000 to OTP_CTRL  (0x2D:06)
     *   c) Wait at least 150 µs
     *   d) Restore clocks:    write 0x0200 to PMSC_CTRL0 (0x36:00)
     */
    uint8_t pmsc_lde[2] = { 0x01, 0x03 };
    DW1000_WriteSubReg(DW_REG_PMSC, DW_SUB_PMSC_CTRL0, pmsc_lde, 2);

    uint8_t otp_ctrl_load[2] = { 0x00, 0x80 };
    DW1000_WriteSubReg(DW_REG_OTP_IF, DW_SUB_OTP_CTRL, otp_ctrl_load, 2);

    HAL_Delay(5);  /* Wait 5ms just to be safe */

    uint8_t pmsc_restore[2] = { 0x00, 0x02 };
    DW1000_WriteSubReg(DW_REG_PMSC, DW_SUB_PMSC_CTRL0, pmsc_restore, 2);

    HAL_Delay(2);

    return 0;  /* Success */
}

/* ========================================================================== */
/*                     CONFIGURATION                                           */
/* ========================================================================== */

/**
 * @brief Configure DW1000 for UWB ranging.
 *
 * Settings (KHỚP với các register được ghi bên dưới):
 *   - Channel:       5 (6489.6 MHz center frequency)
 *   - PRF:           16 MHz
 *   - Preamble:      256 symbols
 *   - Data Rate:     6.8 Mbps
 *   - SFD:           Standard
 *   - PAC:           16
 *   - Preamble Code: 4 (TX and RX)
 *
 * All tuning values are from the DW1000 User Manual.
 * (Trước đây comment ghi nhầm PRF64/preamble128/code9 — đã sửa theo register thật.)
 */
void DW1000_Configure(void)
{
    uint8_t buf[5];

    /* ---- SYS_CFG (0x04): System Configuration ----
     * RXAUTR: automatically resume RX after a hardware frame-filter rejection.
     * FFEN|FFAD: accept only IEEE 802.15.4 data frames addressed to this
     * Anchor/PAN. This prevents every Anchor from leaving RX for the three
     * POLLs and RESP frames that belong to the other Anchors. */
    buf[0] = 0x09; buf[1] = 0x12; buf[2] = 0x04; buf[3] = 0x20;
    DW1000_WriteReg(DW_REG_SYS_CFG, buf, 4);

    /* ---- AGC_TUNE1 (0x23:04): 0x8870 for PRF 16 MHz ---- */
    buf[0] = 0x70; buf[1] = 0x88;
    DW1000_WriteSubReg(DW_REG_AGC_CTRL, DW_SUB_AGC_TUNE1, buf, 2);

    /* ---- AGC_TUNE2 (0x23:0C): 0x2502A907 ---- */
    buf[0] = 0x07; buf[1] = 0xA9; buf[2] = 0x02; buf[3] = 0x25;
    DW1000_WriteSubReg(DW_REG_AGC_CTRL, DW_SUB_AGC_TUNE2, buf, 4);

    /* ---- AGC_TUNE3 (0x23:12): 0x0035 ---- */
    buf[0] = 0x35; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_AGC_CTRL, DW_SUB_AGC_TUNE3, buf, 2);

    /* ---- DRX_TUNE0b (0x27:02): 0x0001 for 6.8Mbps ---- */
    buf[0] = 0x01; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE0b, buf, 2);

    /* ---- DRX_TUNE1a (0x27:04): 0x0087 for PRF 16 MHz ---- */
    buf[0] = 0x87; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE1a, buf, 2);

    /* ---- DRX_TUNE1b (0x27:06): 0x0020 for preamble > 64 ---- */
    buf[0] = 0x20; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE1b, buf, 2);

    /* ---- DRX_TUNE2 (0x27:08): 0x331A0052 for PRF16, PAC16 ---- */
    buf[0] = 0x52; buf[1] = 0x00; buf[2] = 0x1A; buf[3] = 0x33;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE2, buf, 4);

    /* ---- DRX_SFDTOC (0x27:20): 249 for preamble256/SFD8/PAC16 ---- */
    buf[0] = (uint8_t)(DW_PHY_SFD_TIMEOUT & 0xFFU);
    buf[1] = (uint8_t)((DW_PHY_SFD_TIMEOUT >> 8) & 0xFFU);
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_SFDTOC, buf, 2);

    /* ---- DRX_TUNE4H (0x27:26): 0x0028 for preamble >=128 ---- */
    buf[0] = 0x28; buf[1] = 0x00;
    DW1000_WriteSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE4H, buf, 2);

    /* ---- RF_RXCTRLH (0x28:0B): 0xD8 ---- */
    buf[0] = 0xD8;
    DW1000_WriteSubReg(DW_REG_RF_CONF, DW_SUB_RF_RXCTRLH, buf, 1);

    /* ---- RF_TXCTRL (0x28:0C): 0x001E3FE0 for Ch 5 ---- */
    buf[0] = 0xE0; buf[1] = 0x3F; buf[2] = 0x1E; buf[3] = 0x00;
    DW1000_WriteSubReg(DW_REG_RF_CONF, DW_SUB_RF_TXCTRL, buf, 4);

    /* ---- TC_PGDELAY (0x2A:0B): 0xC0 for Ch 5 ---- */
    buf[0] = 0xC0;
    DW1000_WriteSubReg(DW_REG_TX_CAL, DW_SUB_TC_PGDELAY, buf, 1);

    /* ---- FS_PLLCFG (0x2B:07): 0x0800041D for Ch 5 ---- */
    buf[0] = 0x1D; buf[1] = 0x04; buf[2] = 0x00; buf[3] = 0x08;
    DW1000_WriteSubReg(DW_REG_FS_CTRL, DW_SUB_FS_PLLCFG, buf, 4);

    /* ---- FS_PLLTUNE (0x2B:0B): 0xBE for Ch 5 ---- */
    buf[0] = 0xBE;
    DW1000_WriteSubReg(DW_REG_FS_CTRL, DW_SUB_FS_PLLTUNE, buf, 1);

    /* ---- LDE_CFG2 (0x2E:1806): 0x1607 ---- */
    buf[0] = 0x07; buf[1] = 0x16;
    DW1000_WriteSubReg(DW_REG_LDE_IF, DW_SUB_LDE_CFG2, buf, 2);

    /* ---- CHAN_CTRL (0x1F): 0x21040055 ---- */
    /* TX=5, RX=5, RXPRF=16MHz, PCODE=4 */
    buf[0] = 0x55; buf[1] = 0x00; buf[2] = 0x04; buf[3] = 0x21;
    DW1000_WriteReg(DW_REG_CHAN_CTRL, buf, 4);

    /* ---- Antenna Delay (F-01) — có feature flag ---- */
#if UWB_USE_HW_ANTENNA_DELAY
    /* Ghi antenna delay phần cứng (điểm khởi đầu; hiệu chỉnh per-device). */
    buf[0] = (uint8_t)(UWB_TX_ANT_DLY & 0xFF);
    buf[1] = (uint8_t)((UWB_TX_ANT_DLY >> 8) & 0xFF);
    DW1000_WriteReg(DW_REG_TX_ANTD, buf, 2);

    buf[0] = (uint8_t)(UWB_RX_ANT_DLY & 0xFF);
    buf[1] = (uint8_t)((UWB_RX_ANT_DLY >> 8) & 0xFF);
    DW1000_WriteSubReg(DW_REG_LDE_IF, 0x1804, buf, 2);   /* LDE_RXANTD */
#else
    /* Legacy: antenna delay = 0, bù bằng software offset (~157m). */
    buf[0] = 0x00; buf[1] = 0x00;
    DW1000_WriteReg(DW_REG_TX_ANTD, buf, 2);
    DW1000_WriteSubReg(DW_REG_LDE_IF, 0x1804, buf, 2);
#endif

    /* ---- LDE_REPC (0x2E:2804): 0x428E for PCODE 4 ---- */
    buf[0] = 0x8E; buf[1] = 0x42;
    DW1000_WriteSubReg(DW_REG_LDE_IF, DW_SUB_LDE_REPC, buf, 2);

    /* ---- TX_POWER (0x1E): 0x1E1E1E1E for Ch 5 / 16MHz ---- */
    buf[0] = 0x1E; buf[1] = 0x1E; buf[2] = 0x1E; buf[3] = 0x1E;
    DW1000_WriteReg(0x1E, buf, 4);

    /* ---- Wait for the internal clock PLL to lock ---- */
    HAL_Delay(5);

    /* Optionally wait for CPLOCK in SYS_STATUS (bit 1) */
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 100)
    {
        uint8_t status_buf[4];
        DW1000_ReadReg(DW_REG_SYS_STATUS, status_buf, 4);
        uint32_t status = (uint32_t)status_buf[0]
                        | ((uint32_t)status_buf[1] << 8)
                        | ((uint32_t)status_buf[2] << 16)
                        | ((uint32_t)status_buf[3] << 24);
        if (status & (1UL << 1))   /* CPLOCK (Clock PLL lock) */
            break;
        HAL_Delay(1);
    }

    /* ---- SYS_MASK (0x0E): Enable IRQ sources that drive the IRQ pin ----
     * Bit  7 (MTXFRS):  TX Frame Sent
     * Bit 14 (MRXFCG):  RX Frame, FCS Good
     * Bit 15 (MRXFCE):  RX Frame, FCS Error
     * Other error bits (PHY, Reed-Solomon, Preamble timeout) handled
     * by software timeout via HAL_GetTick().
     * HIRQ_POL in SYS_CFG bit9 = 1 → IRQ is active HIGH (rising edge). */
    buf[0] = 0x80;  /* Bit  7: MTXFRS */
    buf[1] = 0xC0;  /* Bit 14: MRXFCG | Bit 15: MRXFCE */
    buf[2] = 0x00;
    buf[3] = 0x00;
    DW1000_WriteReg(DW_REG_SYS_MASK, buf, 4);
}

/* ========================================================================== */
/*                     ADDRESS CONFIGURATION                                   */
/* ========================================================================== */

/**
 * @brief Set the PAN ID and Short Address.
 *
 * PANADR register (0x03) layout (4 bytes, little-endian):
 *   Bytes [1:0] = Short Address
 *   Bytes [3:2] = PAN ID
 */
void DW1000_SetAddress(uint16_t pan_id, uint16_t short_addr)
{
    uint8_t buf[4];
    buf[0] = short_addr & 0xFF;         /* Short address LSB */
    buf[1] = (short_addr >> 8) & 0xFF;  /* Short address MSB */
    buf[2] = pan_id & 0xFF;             /* PAN ID LSB */
    buf[3] = (pan_id >> 8) & 0xFF;      /* PAN ID MSB */
    DW1000_WriteReg(DW_REG_PANADR, buf, 4);
}

/* ========================================================================== */
/*                     TX OPERATIONS                                           */
/* ========================================================================== */

/**
 * @brief Write frame data to the TX buffer.
 */
void DW1000_WriteTxData(const uint8_t *data, uint16_t len)
{
    DW1000_WriteReg(DW_REG_TX_BUFFER, data, len);
}

/**
 * @brief Set the TX Frame Control register.
 *
 * TX_FCTRL (0x08) is 5 bytes (DW_TX_FCTRL_UPPER selects Fast-50 PHY):
 *   Bits [9:0]   = Frame length (including 2-byte FCS)
 *   Bits [14:13] = TXBR  = 10 (6.8 Mbps)
 *   Bit  [15]    = TR    = 1  (Ranging)
 *   Bits [17:16] = TXPRF = 01 (16 MHz)
 *   Bits [19:18] = TXPSR = 01  ┐
 *   Bits [21:20] = PE    = 10  ┴→ Preamble 256 symbols
 *
 * @param len  Total frame length including 2-byte FCS
 */
void DW1000_SetTxFrameCtrl(uint16_t len)
{
    uint32_t fctrl = (uint32_t)len | DW_TX_FCTRL_UPPER;
    uint8_t buf[5];
    buf[0] = fctrl & 0xFF;
    buf[1] = (fctrl >> 8) & 0xFF;
    buf[2] = (fctrl >> 16) & 0xFF;
    buf[3] = (fctrl >> 24) & 0xFF;
    buf[4] = 0x00;  /* TX_FCTRL byte 4: IFSDELAY, leave at 0 */
    DW1000_WriteReg(DW_REG_TX_FCTRL, buf, 5);
}

/**
 * @brief Start transmission (set TXSTRT bit in SYS_CTRL).
 */
void DW1000_StartTx(void)
{
    uint8_t buf[1] = { 0x02 };  /* TXSTRT = bit 1 */
    DW1000_WriteReg(DW_REG_SYS_CTRL, buf, 1);
}

/**
 * @brief Set the delayed TX time (writes to DX_TIME register 0x0A).
 *
 * @param tx_time  5-byte delayed transmit time (40-bit DW1000 timestamp)
 */
void DW1000_SetDelayedTxTime(const uint8_t tx_time[5])
{
    DW1000_WriteReg(DW_REG_DX_TIME, tx_time, 5);
}

/**
 * @brief Start a delayed transmission (sets TXSTRT + TXDLYS bits) và kiểm tra
 *        HPDWARN ngay lập tức.
 *
 * Must call DW1000_SetDelayedTxTime() first to set the scheduled time.
 * Nếu thời điểm phát đã trôi qua (SPI/xử lý quá chậm), DW1000 set HPDWARN và
 * TX sẽ KHÔNG xảy ra → phải phát hiện ngay để recovery, tránh chờ TX timeout (F-09).
 *
 * @retval 0  = đã lên lịch TX
 * @retval -1 = TRỄ (HPDWARN)
 */
int DW1000_StartTxDelayed(void)
{
    uint8_t buf[1] = { 0x06 };  /* TXSTRT (bit 1) + TXDLYS (bit 2) */
    DW1000_WriteReg(DW_REG_SYS_CTRL, buf, 1);

    uint32_t status = DW1000_ReadStatus();
    if (status & DW_HPDWARN_BIT)
        return -1;   /* delayed time đã qua — caller recovery ngay */

    return 0;
}

/**
 * @brief Wait for transmission to complete.
 *
 * Polls SYS_STATUS register for TXFRS (TX Frame Sent) bit.
 *
 * @param timeout_ms  Maximum wait time
 * @retval 1 = TX done, 0 = timeout
 */
int DW1000_WaitTxDone(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        uint32_t status = DW1000_ReadStatus();
        if (status & DW_TXFRS_BIT)
        {
            /* Clear the TX status bits */
            uint8_t buf[4] = { 0xF8, 0x00, 0x00, 0x00 };  /* Clear bits [7:3] — all TX events */
            DW1000_WriteReg(DW_REG_SYS_STATUS, buf, 4);
            return 1;  /* TX done */
        }
    }

    return 0;  /* Timeout */
}

/* ========================================================================== */
/*                     RX OPERATIONS                                           */
/* ========================================================================== */

/**
 * @brief Enable the receiver (set RXENAB bit in SYS_CTRL).
 *
 * RXENAB is bit 8 of SYS_CTRL. We write 2 bytes: [0x00, 0x01]
 * to set only bit 8 without disturbing other bits.
 */
void DW1000_StartRx(void)
{
    uint8_t buf[2] = { 0x00, 0x01 };  /* Bit 8 = RXENAB */
    DW1000_WriteReg(DW_REG_SYS_CTRL, buf, 2);
}

/**
 * @brief Wait for a frame to be received.
 *
 * Polls SYS_STATUS for:
 *   - RXDFR + RXFCG: Good frame received → return 1
 *   - RXFCE, RXRFTO, RXPTO: RX error → return 2
 *   - Neither within timeout → return 0
 *
 * @param timeout_ms  Software timeout in milliseconds
 * @retval 0 = timeout, 1 = good frame, 2 = error
 */
int DW1000_WaitRxDone(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        uint32_t status = DW1000_ReadStatus();

        /* Check for good frame (both RXDFR and RXFCG set) */
        if ((status & DW_ALL_RX_GOOD) == DW_ALL_RX_GOOD)
        {
            return 1;  /* Good frame received */
        }

        /* Check for any RX error */
        if (status & DW_ALL_RX_ERR)
        {
            return 2;  /* RX error */
        }
    }

    return 0;  /* Software timeout */
}

/**
 * @brief Read received frame data from the RX buffer.
 *
 * First reads RX_FINFO to determine the actual frame length,
 * then reads that many bytes from RX_BUFFER.
 *
 * @param data     Buffer to store the received frame
 * @param max_len  Maximum bytes the buffer can hold
 * @retval Actual number of bytes read
 */
uint16_t DW1000_ReadRxData(uint8_t *data, uint16_t max_len)
{
    /* Read RX_FINFO (4 bytes) to get frame length */
    uint8_t finfo[4];
    DW1000_ReadReg(DW_REG_RX_FINFO, finfo, 4);

    /* Frame length is in bits [6:0] of RX_FINFO */
    uint16_t frame_len = finfo[0] & 0x7F;

    /* Clamp to max_len */
    if (frame_len > max_len)
        frame_len = max_len;

    if (frame_len > 0)
    {
        DW1000_ReadReg(DW_REG_RX_BUFFER, data, frame_len);
    }

    return frame_len;
}

/**
 * @brief Read the 5-byte RX timestamp from RX_TIME register.
 *
 * The timestamp is a 40-bit counter value at ~15.65 ps resolution
 * (~64 GHz clock), representing when the first path of the received
 * frame was detected.
 */
void DW1000_ReadRxTimestamp(uint8_t ts[5])
{
    DW1000_ReadReg(DW_REG_RX_TIME, ts, 5);
}

/**
 * @brief Read the 5-byte TX timestamp from TX_TIME register.
 *
 * The timestamp is a 40-bit counter value at ~15.65 ps resolution,
 * representing when the RMARKER of the transmitted frame left the antenna.
 */
void DW1000_ReadTxTimestamp(uint8_t ts[5])
{
    DW1000_ReadReg(DW_REG_TX_TIME, ts, 5);
}

/* ========================================================================== */
/*                     STATUS / CONTROL                                        */
/* ========================================================================== */

/**
 * @brief Clear all event flags in SYS_STATUS.
 *
 * Writing 1s to SYS_STATUS clears the corresponding event bits.
 */
void DW1000_ClearAllStatus(void)
{
    uint8_t buf[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    DW1000_WriteReg(DW_REG_SYS_STATUS, buf, 4);
}

/**
 * @brief Force the transceiver off (abort any TX or RX in progress).
 *
 * Sets the TRXOFF bit (bit 6) in SYS_CTRL.
 */
void DW1000_ForceRxOff(void)
{
    uint8_t buf[1] = { 0x40 };  /* TRXOFF = bit 6 */
    DW1000_WriteReg(DW_REG_SYS_CTRL, buf, 1);
    
    /* DW1000 needs up to 40us to return to IDLE after TRXOFF.
     * 1500 volatile iterations @ 64MHz is ~100us, safely ensuring IDLE
     * before any subsequent TX or RX commands. */
    for (volatile int i = 0; i < 1500; i++) {
        __NOP();
    }
}

/**
 * @brief Read the SYS_STATUS register (first 4 bytes of the 5-byte register).
 * @retval 32-bit status value
 */
uint32_t DW1000_ReadStatus(void)
{
    uint8_t buf[4];
    DW1000_ReadReg(DW_REG_SYS_STATUS, buf, 4);

    return (uint32_t)buf[0]
         | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16)
         | ((uint32_t)buf[3] << 24);
}

/* ========================================================================== */
/*                     INTERRUPT SUPPORT                                       */
/* ========================================================================== */

/** IRQ flag — set by EXTI callback, cleared by Anchor_Task / Tag_Task */
volatile uint8_t dw1000_irq_flag = 0;

/**
 * @brief  Reconfigure PA3 (DW1000 IRQ) as rising-edge EXTI and enable NVIC.
 *
 * Call order:
 *   DW1000_Init()         — must complete first (SPI + reset + LDE)
 *   DW1000_Configure()    — writes SYS_MASK to arm IRQ sources
 *   DW1000_ClearAllStatus() — ensures IRQ pin is LOW before EXTI armed
 *   DW1000_EnableIRQ()    — arm EXTI (safe, no spurious fire)
 */
void DW1000_EnableIRQ(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = DW_IRQ_PIN;           /* PA3 */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  /* Rising edge (HIRQ_POL=1) */
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;         /* Keep LOW when idle */
    HAL_GPIO_Init(DW_IRQ_PORT, &GPIO_InitStruct);

    /* Priority 0 (highest) — ISR only sets a flag, very short */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

/**
 * @brief  EXTI callback called by HAL from EXTI3_IRQHandler.
 *         ONLY sets the flag — no SPI, no computation here.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DW_IRQ_PIN)
    {
        dw1000_irq_flag = 1;
    }
}

/* ========================================================================== */
/*                 PHASE 2: µs TIMER (DWT), IRQ LEVEL, VERIFY                  */
/* ========================================================================== */

static uint32_t s_cycles_per_us = 0;   /* FIX-02: cache 1 lần trong TimerInit */

/** Bật DWT cycle counter làm đồng hồ µs. */
void MCU_TimerInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* FIX-02: cache để MCU_ElapsedUs() không chia lại mỗi lần gọi */
    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_cycles_per_us == 0U) s_cycles_per_us = 1U;
}

/** Micro-giây kể từ MCU_TimerInit(). CHỈ dùng cho log tuyệt đối (FIX-02). */
uint32_t MCU_Micros(void)
{
    return DWT->CYCCNT / (s_cycles_per_us ? s_cycles_per_us : 1U);
}

/** FIX-02: đọc mốc cycle thô hiện tại. */
McuCycleStamp_t MCU_CycleNow(void)
{
    return DWT->CYCCNT;
}

/** FIX-02: elapsed µs an toàn qua wrap — trừ RAW cycle trước khi đổi µs. */
uint32_t MCU_ElapsedUs(McuCycleStamp_t start)
{
    uint32_t elapsed_cycles = (uint32_t)(DWT->CYCCNT - start);
    return elapsed_cycles / (s_cycles_per_us ? s_cycles_per_us : 1U);
}

uint8_t DW1000_IrqLineActive(void)
{
    return (HAL_GPIO_ReadPin(DW_IRQ_PORT, DW_IRQ_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

/* Giá trị kỳ vọng của các thanh ghi PHY bền vững (ghi trong DW1000_Configure). */
#define DW_EXP_SYS_CFG      0x20041209UL   /* RXAUTR|DIS_DRXB|HIRQ_POL|DIS_STXP|FFAD|FFEN */
#define DW_EXP_CHAN_CTRL    0x21040055UL   /* Ch5, RXPRF16, PCODE4 */

#define DW_VERIFY_DEVID     0x01
#define DW_VERIFY_SYS_CFG   0x02
#define DW_VERIFY_CHAN_CTRL 0x04
#define DW_VERIFY_SFDTOC    0x08
#define DW_VERIFY_DRX_TUNE2 0x10

uint32_t DW1000_VerifyConfig(void)
{
    uint32_t mismatch = 0;
    uint8_t buf[4];

    if (DW1000_ReadDeviceID() != DW1000_DEVICE_ID)
        mismatch |= DW_VERIFY_DEVID;

    DW1000_ReadReg(DW_REG_SYS_CFG, buf, 4);
    uint32_t sys_cfg = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                     | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    if (sys_cfg != DW_EXP_SYS_CFG)
        mismatch |= DW_VERIFY_SYS_CFG;

    DW1000_ReadReg(DW_REG_CHAN_CTRL, buf, 4);
    uint32_t chan = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                  | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    if (chan != DW_EXP_CHAN_CTRL)
        mismatch |= DW_VERIFY_CHAN_CTRL;

    DW1000_ReadSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_SFDTOC, buf, 2);
    uint16_t sfdtoc = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    if (sfdtoc != (uint16_t)DW_PHY_SFD_TIMEOUT)
        mismatch |= DW_VERIFY_SFDTOC;

    DW1000_ReadSubReg(DW_REG_DRX_CONF, DW_SUB_DRX_TUNE2, buf, 4);
    uint32_t drx_tune2 = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8)
                       | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    if (drx_tune2 != 0x331A0052UL)
        mismatch |= DW_VERIFY_DRX_TUNE2;

    return mismatch;
}
