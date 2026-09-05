/**
  ******************************************************************************
  * @file    dw1000_hw.h
  * @brief   DW1000 UWB Hardware Driver — Shared Header
  *
  * Pin mapping (from schematic):
  *   PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI
  *   PA2=RSTN, PA3=IRQ, PA1=EXTON, PB0=WAKE
  *   PC13=LED (active LOW, Blue Pill)
  *
  * This header is shared between Anchor and Tag projects.
  ******************************************************************************
  */

#ifndef DW1000_HW_H
#define DW1000_HW_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "uwb_calibration.h"

extern SPI_HandleTypeDef hspi1;

/* ========================================================================== */
/*                          PIN DEFINITIONS                                    */
/* ========================================================================== */

/* SPI pins (bit-bang) */
#define SPI_CS_PORT         GPIOA
#define SPI_CS_PIN          GPIO_PIN_4
#define SPI_SCK_PORT        GPIOA
#define SPI_SCK_PIN         GPIO_PIN_5
#define SPI_MISO_PORT       GPIOA
#define SPI_MISO_PIN        GPIO_PIN_6
#define SPI_MOSI_PORT       GPIOA
#define SPI_MOSI_PIN        GPIO_PIN_7

/* DW1000 control pins */
#define DW_RSTN_PORT        GPIOA
#define DW_RSTN_PIN         GPIO_PIN_2
#define DW_IRQ_PORT         GPIOA
#define DW_IRQ_PIN          GPIO_PIN_3
#define DW_EXTON_PORT       GPIOA
#define DW_EXTON_PIN        GPIO_PIN_1
#define DW_WAKE_PORT        GPIOB
#define DW_WAKE_PIN         GPIO_PIN_0

/* LED pin (active LOW on Blue Pill) */
#define LED_PORT            GPIOC
#define LED_PIN             GPIO_PIN_13

/* ========================================================================== */
/*                     DW1000 REGISTER MAP                                     */
/* ========================================================================== */

#define DW_REG_DEV_ID       0x00    /* Device Identifier (4 bytes, RO) */
#define DW_REG_EUI          0x01    /* Extended Unique Identifier (8 bytes) */
#define DW_REG_PANADR       0x03    /* PAN ID and Short Address (4 bytes) */
#define DW_REG_SYS_CFG      0x04    /* System Configuration (4 bytes) */
#define DW_REG_SYS_TIME     0x06    /* System Time Counter (5 bytes, RO) */
#define DW_REG_TX_FCTRL     0x08    /* TX Frame Control (5 bytes) */
#define DW_REG_TX_BUFFER    0x09    /* TX Data Buffer (up to 1024 bytes) */
#define DW_REG_DX_TIME      0x0A    /* Delayed Send or Receive Time (5 bytes) */
#define DW_REG_SYS_CTRL     0x0D    /* System Control Register (4 bytes) */
#define DW_REG_SYS_MASK     0x0E    /* System Event Mask Register (4 bytes) */
#define DW_REG_SYS_STATUS   0x0F    /* System Event Status Register (5 bytes) */
#define DW_REG_RX_FINFO     0x10    /* RX Frame Information (4 bytes, RO) */
#define DW_REG_RX_BUFFER    0x11    /* RX Data Buffer (up to 1024 bytes, RO) */
#define DW_REG_RX_TIME      0x15    /* RX Message Time of Arrival (14 bytes, RO) */
#define DW_REG_TX_TIME      0x17    /* TX Message Time of Sending (10 bytes, RO) */
#define DW_REG_TX_ANTD      0x18    /* TX Antenna Delay (2 bytes) */
#define DW_REG_CHAN_CTRL     0x1F    /* Channel Control (4 bytes) */
#define DW_REG_AGC_CTRL     0x23    /* AGC configuration and control block */
#define DW_REG_DRX_CONF     0x27    /* Digital Receiver configuration block */
#define DW_REG_RF_CONF      0x28    /* Analog RF configuration block */
#define DW_REG_TX_CAL       0x2A    /* Transmitter Calibration block */
#define DW_REG_FS_CTRL      0x2B    /* Frequency Synthesiser control block */
#define DW_REG_OTP_IF       0x2D    /* One-Time Programmable Memory IF */
#define DW_REG_LDE_IF       0x2E    /* Leading Edge Detection IF */
#define DW_REG_PMSC         0x36    /* Power Management and System Control */

/* ========================================================================== */
/*                     SUB-REGISTER OFFSETS                                    */
/* ========================================================================== */

/* AGC_CTRL sub-registers */
#define DW_SUB_AGC_TUNE1    0x04    /* AGC Tuning register 1 (2 bytes) */
#define DW_SUB_AGC_TUNE2    0x0C    /* AGC Tuning register 2 (4 bytes) */
#define DW_SUB_AGC_TUNE3    0x12    /* AGC Tuning register 3 (2 bytes) */

/* DRX_CONF sub-registers */
#define DW_SUB_DRX_TUNE0b   0x02   /* Digital Tuning 0b (2 bytes) */
#define DW_SUB_DRX_TUNE1a   0x04   /* Digital Tuning 1a (2 bytes) */
#define DW_SUB_DRX_TUNE1b   0x06   /* Digital Tuning 1b (2 bytes) */
#define DW_SUB_DRX_TUNE2    0x08   /* Digital Tuning 2 (4 bytes) */
#define DW_SUB_DRX_SFDTOC   0x20   /* SFD detection timeout (2 bytes) */
#define DW_SUB_DRX_TUNE4H   0x26   /* Digital Tuning 4H (2 bytes) */

/* RF_CONF sub-registers */
#define DW_SUB_RF_RXCTRLH   0x0B   /* RF RX Control (1 byte) */
#define DW_SUB_RF_TXCTRL    0x0C   /* RF TX Control (4 bytes) */

/* TX_CAL sub-registers */
#define DW_SUB_TC_PGDELAY   0x0B   /* Pulse Generator Delay (1 byte) */

/* FS_CTRL sub-registers */
#define DW_SUB_FS_PLLCFG    0x07   /* PLL Configuration (4 bytes) */
#define DW_SUB_FS_PLLTUNE   0x0B   /* PLL Tuning (1 byte) */

/* LDE_IF sub-registers (note: some use extended addressing) */
#define DW_SUB_LDE_CFG2     0x1806 /* LDE Configuration 2 (2 bytes) — extended */
#define DW_SUB_LDE_REPC     0x2804 /* LDE Replica Coefficient (2 bytes) — extended */

/* OTP_IF sub-registers */
#define DW_SUB_OTP_CTRL     0x06   /* OTP Control (2 bytes) */

/* PMSC sub-registers */
#define DW_SUB_PMSC_CTRL0   0x00   /* PMSC Control 0 (4 bytes) */

/* ========================================================================== */
/*                     SYS_STATUS BIT MASKS                                    */
/* ========================================================================== */

#define DW_TXFRS_BIT        (1UL << 7)   /* TX Frame Sent */
#define DW_RXPHD_BIT        (1UL << 11)  /* RX PHY Header Detected */
#define DW_RXPHE_BIT        (1UL << 12)  /* RX PHY Header Error */
#define DW_RXDFR_BIT        (1UL << 13)  /* RX Data Frame Ready */
#define DW_RXFCG_BIT        (1UL << 14)  /* RX FCS Good */
#define DW_RXFCE_BIT        (1UL << 15)  /* RX FCS Error */
#define DW_RXRFSL_BIT       (1UL << 16)  /* RX Reed Solomon Frame Sync Loss */
#define DW_RXRFTO_BIT       (1UL << 17)  /* RX Frame Wait Timeout */
#define DW_RXPTO_BIT        (1UL << 21)  /* Preamble Detection Timeout */
#define DW_TXPUTE_BIT       (1UL << 26)  /* TX Power Up Time Error (delayed TX) */
#define DW_HPDWARN_BIT      (1UL << 27)  /* Half Period Delay Warning (delayed TX late) */

/* Combined masks for convenience */
#define DW_ALL_RX_GOOD      (DW_RXDFR_BIT | DW_RXFCG_BIT)
#define DW_ALL_RX_ERR       (DW_RXFCE_BIT | DW_RXRFTO_BIT | DW_RXPTO_BIT | DW_RXPHE_BIT | DW_RXRFSL_BIT)
#define DW_ALL_TX_DONE      (DW_TXFRS_BIT)

/* ========================================================================== */
/*                     SYS_CTRL BIT MASKS                                      */
/* ========================================================================== */

#define DW_TXSTRT_BIT       (1UL << 1)   /* Transmit Start */
#define DW_TRXOFF_BIT       (1UL << 6)   /* Force TRX Off */
#define DW_RXENAB_BIT       (1UL << 8)   /* Enable Receiver */

/* ========================================================================== */
/*                     DEVICE ID                                               */
/* ========================================================================== */

#define DW1000_DEVICE_ID    0xDECA0130UL

/* ========================================================================== */
/*                     FRAME / PROTOCOL CONSTANTS                              */
/* ========================================================================== */

#define FRAME_POLL_FUNC     0x21    /* Function code: Poll message */
#define FRAME_RESP_FUNC     0x10    /* Function code: Response message */
#define FRAME_REPORT_FUNC   0x22    /* Function code: Report message (Distance) */
#define FRAME_FINAL_FUNC    0x23    /* Function code: Final message (future use) */

#define DW_PAN_ID           0xDECA  /* PAN Identifier */
#define ANCHOR_ADDR         0x0003  /* Anchor 3 short address */
#define TAG_ADDR            0x0000  /* Tag short address — PHẢI khớp TAG project (TAG phát src=0x0000).
                                       Phase 4 WAIT_FINAL so src_addr==TAG_ADDR; giá trị 0x0002 cũ là
                                       rác không ai dùng trước đây. */

/* Fast-50 profile: Ch5, 6.8Mbps, PRF16, preamble 256, PAC16, standard SFD.
 * DWT_PLEN_256 encoding is 0x24 in TX_FCTRL[21:18]. */
#define DW_PHY_PREAMBLE_SYMBOLS  256U
#define DW_PHY_PAC_SYMBOLS       16U
#define DW_PHY_SFD_TIMEOUT       249U  /* preamble + 1 + SFD(8) - PAC(16) */
#define DW_PHY_PROFILE_ID        2U    /* 0=unspecified, 1=legacy1024, 2=Fast-256 */
#define DW_TX_FCTRL_UPPER        0x0025C000UL

/* Runtime SPI starts at 2MHz for reset/OTP/LDE, then moves to 16MHz only
 * after DW1000 initialization. The driver automatically falls back to 8/2MHz
 * if the device-ID readback is not reliable at the faster rate. */
#define DW_SPI_INIT_MHZ          2U
#define DW_SPI_FAST_MHZ          16U
extern volatile uint8_t dw1000_spi_mhz;

/* Default antenna delay (calibration value for ~0.5m accuracy) */
#define DW_DEFAULT_ANT_DLY  16436   /* 0x4024 */

/* ========================================================================== */
/*                     RANGING RESULT STRUCTURE                                */
/* ========================================================================== */

/**
 * @brief Ranging result data — designed to be watched in the debugger
 */
typedef struct {
    int32_t  distance_mm;       /* Computed distance in millimeters */
    int32_t  distance_cm;       /* Computed distance in centimeters */
    uint8_t  poll_rx_ts[5];     /* Anchor: Poll RX timestamp (40-bit) */
    uint8_t  resp_tx_ts[5];     /* Anchor: Response TX timestamp (40-bit) */
    uint8_t  poll_tx_ts[5];     /* Tag: Poll TX timestamp (40-bit) */
    uint8_t  resp_rx_ts[5];     /* Tag: Response RX timestamp (40-bit) */
    uint32_t ranging_count;     /* Total successful ranging cycles */
    uint32_t timeout_count;     /* Total RX timeouts */
    uint32_t last_status;       /* Last SYS_STATUS value for debugging */
} DW1000_RangingResult;

/* ========================================================================== */
/*                     API FUNCTION PROTOTYPES                                 */
/* ========================================================================== */

/**
 * @brief  Initialize the DW1000: GPIO setup, hardware reset, verify Device ID,
 *         and load LDE microcode from OTP memory.
 * @retval 0 = success, -1 = device ID mismatch
 */
int DW1000_Init(void);

/**
 * @brief  Switch SPI from the <=3MHz initialization rate to the runtime rate.
 *         Call only after DW1000_Init() and DW1000_Configure().
 * @return Selected rate in MHz (16, 8, or 2); 0 if even the safe rate fails.
 */
uint8_t DW1000_EnableFastSPI(void);

/**
 * @brief  Configure DW1000 for ranging on Channel 5, PRF 64MHz,
 *         Preamble 128, Data Rate 6.8Mbps.
 */
void DW1000_Configure(void);

/**
 * @brief  Set the PAN ID and Short Address in DW1000's PANADR register.
 * @param  pan_id  16-bit PAN identifier
 * @param  short_addr  16-bit short address
 */
void DW1000_SetAddress(uint16_t pan_id, uint16_t short_addr);

/**
 * @brief  Write data to the TX buffer.
 * @param  data  Pointer to frame data
 * @param  len   Number of bytes to write
 */
void DW1000_WriteTxData(const uint8_t *data, uint16_t len);

/**
 * @brief  Set the TX Frame Control register (frame length + config).
 * @param  len  Total frame length including 2-byte FCS
 */
void DW1000_SetTxFrameCtrl(uint16_t len);

/**
 * @brief  Start transmission by setting TXSTRT bit in SYS_CTRL.
 */
void DW1000_StartTx(void);

/**
 * @brief  Set the delayed TX time (DX_TIME register).
 * @param  tx_time  5-byte delayed transmit time (40-bit DW1000 timestamp)
 */
void DW1000_SetDelayedTxTime(const uint8_t tx_time[5]);

/**
 * @brief  Start delayed transmission (sets TXSTRT + TXDLYS bits) và kiểm tra
 *         ngay HPDWARN — nếu thời điểm phát đã trôi qua, TX sẽ không xảy ra.
 * @retval 0  = đã lên lịch TX thành công
 * @retval -1 = TRỄ (HPDWARN set) — caller phải recovery ngay, không chờ timeout (F-09)
 */
int DW1000_StartTxDelayed(void);

/**
 * @brief  Wait for TX to complete (poll SYS_STATUS for TXFRS).
 * @param  timeout_ms  Maximum wait time in milliseconds
 * @retval 1 = TX done, 0 = timeout
 */
int DW1000_WaitTxDone(uint32_t timeout_ms);

/**
 * @brief  Enable the receiver by setting RXENAB in SYS_CTRL.
 */
void DW1000_StartRx(void);

/**
 * @brief  Wait for a frame to be received.
 * @param  timeout_ms  Maximum wait time in milliseconds
 * @retval 0 = timeout, 1 = good frame, 2 = RX error
 */
int DW1000_WaitRxDone(uint32_t timeout_ms);

/**
 * @brief  Read received frame data from RX buffer.
 * @param  data     Buffer to store received data
 * @param  max_len  Maximum bytes to read
 * @retval Actual number of bytes read (from RX_FINFO)
 */
uint16_t DW1000_ReadRxData(uint8_t *data, uint16_t max_len);

/**
 * @brief  Read the 5-byte RX timestamp from RX_TIME register.
 * @param  ts  5-byte buffer to store the timestamp
 */
void DW1000_ReadRxTimestamp(uint8_t ts[5]);

/**
 * @brief  Read the 5-byte TX timestamp from TX_TIME register.
 * @param  ts  5-byte buffer to store the timestamp
 */
void DW1000_ReadTxTimestamp(uint8_t ts[5]);

/**
 * @brief  Clear all status flags by writing 0xFFFFFFFF to SYS_STATUS.
 */
void DW1000_ClearAllStatus(void);

/**
 * @brief  Force receiver off by setting TRXOFF in SYS_CTRL.
 */
void DW1000_ForceRxOff(void);

/**
 * @brief  Read the SYS_STATUS register (4 bytes).
 * @retval 32-bit status value
 */
uint32_t DW1000_ReadStatus(void);

/**
 * @brief  Read the DW1000 Device ID register.
 * @retval 32-bit device ID (should be 0xDECA0130)
 */
uint32_t DW1000_ReadDeviceID(void);

/**
 * @brief  Configure the DW1000 IRQ pin (PA3) as a rising-edge EXTI interrupt
 *         and enable it in NVIC. Must be called AFTER DW1000_Init() +
 *         DW1000_Configure() + DW1000_ClearAllStatus().
 *
 * When DW1000 signals TX done or RX done, it drives IRQ (PA3) HIGH.
 * The EXTI ISR sets dw1000_irq_flag = 1.
 * Application code must poll this flag and clear it.
 */
void DW1000_EnableIRQ(void);

/**
 * @brief  IRQ flag set by EXTI3 interrupt handler (HAL_GPIO_EXTI_Callback).
 *         Set to 1 on DW1000 IRQ rising edge.
 *         Must be cleared (set to 0) by the application after handling.
 */
extern volatile uint8_t dw1000_irq_flag;

/* ========================================================================== */
/*                     PHASE 2: µs TIMER, IRQ LEVEL, CONFIG VERIFY             */
/* ========================================================================== */

/** Bật DWT cycle counter làm đồng hồ µs. Gọi 1 lần sau SystemClock_Config(). */
void MCU_TimerInit(void);

/** Micro-giây kể từ MCU_TimerInit() (wrap ~67s @64MHz).
 *  CHỈ dùng cho giá trị tuyệt đối/log. KHÔNG dùng để tính delta/elapsed —
 *  dùng MCU_CycleNow() + MCU_ElapsedUs() cho mục đích đó (FIX-02). */
uint32_t MCU_Micros(void);

/** FIX-02: kiểu dữ liệu "mốc cycle thô" — snapshot CYCCNT thô, dùng làm
 *  điểm bắt đầu cho MCU_ElapsedUs(). KHÔNG phải microsecond. */
typedef uint32_t McuCycleStamp_t;

/** FIX-02: Đọc mốc cycle thô hiện tại (CYCCNT). */
McuCycleStamp_t MCU_CycleNow(void);

/** FIX-02: Elapsed µs an toàn qua wrap CYCCNT (~67s @64MHz).
 *  Trừ RAW CYCCNT (uint32_t modulo 2^32) TRƯỚC khi quy đổi sang µs. */
uint32_t MCU_ElapsedUs(McuCycleStamp_t start);

/** Đọc mức chân IRQ (PA3): 1 nếu còn event chưa xử lý, 0 nếu LOW (bổ sung F). */
uint8_t DW1000_IrqLineActive(void);

/** Đọc lại register PHY quan trọng, so với kỳ vọng. 0 = khớp (F-03). */
uint32_t DW1000_VerifyConfig(void);

#endif /* DW1000_HW_H */
