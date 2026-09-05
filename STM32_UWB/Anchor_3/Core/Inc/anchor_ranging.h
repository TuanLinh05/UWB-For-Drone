/**
 ******************************************************************************
 * @file    anchor_ranging.h
 * @brief   Anchor SS-TWR Ranging — Interrupt-driven State Machine Header
 *
 * Architecture change: No more blocking waits.
 * - Anchor_Init()           : One-time init.
 * - Anchor_StartListening() : Arms RX for the first time.
 * - Anchor_Task()           : Call repeatedly from main while(1).
 *                             Returns immediately if no IRQ pending.
 *                             Drives the full POLL→RESP state machine.
 ******************************************************************************
 */

#ifndef ANCHOR_RANGING_H
#define ANCHOR_RANGING_H

#include "dw1000_hw.h"

/* ========================================================================== */
/*                     TIMING CONSTANTS                                        */
/* ========================================================================== */

/**
 * Reply delay from POLL RX to RESPONSE TX, in UWB microseconds.
 * 2500 UUS ≈ 5.0 ms — long enough for SW processing, short enough to
 * minimize clock-drift error in SS-TWR.
 */
#define ANCHOR_REPLY_DELAY_UUS  1200UL

/**
 * Maximum time allowed for a delayed TX to complete (backstop cho treo phần cứng).
 * Phase 2: late-TX đã được bắt ngay bằng HPDWARN (F-09), nên chỉ cần backstop nhỏ.
 * Giảm 50→10ms để một lỗi TX không nuốt nhiều chu kỳ.
 */
#define ANCHOR_TX_TIMEOUT_MS    10

/** DS-TWR: max wait for FINAL from TAG after RESP TX (ms).
 *  FINAL must arrive ~1-2ms; 5ms margin. Must be SMALLER than inter-POLL interval
 *  (~5-7ms) so anchor returns to RX_WAIT before TAG's next POLL cycle. */
#define ANCHOR_FINAL_TIMEOUT_MS  5

/** Đếm số lần delayed-TX bị trễ (HPDWARN) — Live Expressions (F-09). */
extern volatile uint32_t anchor_delayed_tx_late_count;

/* ========================================================================== */
/*                     FUNCTION PROTOTYPES                                     */
/* ========================================================================== */

/**
 * @brief  Initialize the Anchor: DW1000 init + configure + set address.
 *         SYS_MASK is set inside DW1000_Configure() to arm TX/RX IRQ sources.
 * @retval 0 = success, -1 = DW1000 init failure
 */
int Anchor_Init(void);

/**
 * @brief  Start listening for POLL frames (enables DW1000 RX once).
 *         Call AFTER DW1000_EnableIRQ().
 */
void Anchor_StartListening(void);

/**
 * @brief  Non-blocking state machine tick.
 *         Must be called continuously from the main while(1) loop.
 *
 *         State transitions:
 *           RX_WAIT  --[IRQ: RX_GOOD + valid POLL]--> TX_RESPOND
 *           TX_RESPOND --[IRQ: TX_DONE]            --> RX_WAIT
 *           TX_RESPOND --[timeout 50ms]             --> RX_WAIT (error recovery)
 *           RX_WAIT  --[IRQ: RX_ERROR]             --> RX_WAIT (restart RX)
 */
void Anchor_Task(void);

/**
 * @brief  Get a pointer to the ranging result structure.
 * @retval Pointer to DW1000_RangingResult (static, always valid)
 */
const DW1000_RangingResult* Anchor_GetResult(void);

#endif /* ANCHOR_RANGING_H */
