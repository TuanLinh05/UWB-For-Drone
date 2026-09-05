/**
 ******************************************************************************
 * @file    anchor_ranging.c
 * @brief   Anchor TWR Ranging — Interrupt-driven State Machine
 *
 * Protocol (SS-TWR, default UWB_USE_DS_TWR=0):
 *   1. TAG  ──[POLL]──> Anchor   (broadcast to specific anchor address)
 *   2. Anchor ──[RESP]──> TAG    (delayed TX, T_reply embedded in payload)
 *   3. TAG computes: ToF = (T_round - T_reply) / 2
 *
 * Protocol (DS-TWR, UWB_USE_DS_TWR=1):
 *   1. TAG  ──[POLL]──>   Anchor
 *   2. Anchor ──[RESP]──> TAG     (delayed TX, T_reply=Da in payload)
 *   3. TAG  ──[FINAL]──>  Anchor  (immediate TX)
 *   4. Anchor ──[REPORT]──> TAG   (immediate TX, Rb in payload)
 *   TAG computes: tof = (Ra*Rb - Da*Db) / (Ra+Rb+Da+Db)
 *
 * Frame format (IEEE 802.15.4 short address, PAN ID compression):
 *   [0-1]  Frame Control: 0x41, 0x88
 *   [2]    Sequence Number
 *   [3-4]  PAN ID: 0xCA, 0xDE (little-endian 0xDECA)
 *   [5-6]  Destination Address (little-endian)
 *   [7-8]  Source Address (little-endian)
 *   [9]    Function Code
 *   [10-13] Payload (RESP: T_reply ticks; REPORT: Rb ticks — 32-bit LSB-first)
 *
 * Key design principle:
 *   The DW1000 IRQ pin (PA3) fires on TX_DONE and RX_DONE events.
 *   The EXTI3 ISR sets dw1000_irq_flag = 1 and returns immediately.
 *   All SPI transactions and logic run here in Anchor_Task(),
 *   called from the main while(1) loop — never from within the ISR.
 ******************************************************************************
 */

#include "anchor_ranging.h"
#include <string.h>

/* ========================================================================== */
/*                     PRIVATE DEFINES                                         */
/* ========================================================================== */

#define MAX_RX_FRAME_LEN    20
#define RESP_FRAME_LEN      14  /* FC(2)+Seq(1)+PAN(2)+Dst(2)+Src(2)+Func(1)+T_reply(4) */
#define REPORT_FRAME_LEN    14  /* Same structure, Func=FRAME_REPORT_FUNC, payload=Rb */

#define UUS_TO_DWT_TIME     65536ULL
#define REPLY_DELAY_TICKS   ((uint64_t)ANCHOR_REPLY_DELAY_UUS * UUS_TO_DWT_TIME)

#define LED_TOGGLE()        HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

/* FIX-01: TX antenna delay đang active trong thanh ghi TX_ANTD — phải khớp
 * với những gì DW1000_Configure() (dw1000_hw.c) đã ghi.
 * = 0 khi UWB_USE_HW_ANTENNA_DELAY=0 (mặc định hiện tại) → t_reply KHÔNG
 * đổi so với trước khi sửa (bất biến, hành vi đo giữ nguyên ở default). */
#if UWB_USE_HW_ANTENNA_DELAY
    #define UWB_ACTIVE_TX_ANT_DLY   ((uint64_t)UWB_TX_ANT_DLY)
#else
    #define UWB_ACTIVE_TX_ANT_DLY   ((uint64_t)0)
#endif

/* Mask 40-bit timestamp (chống underflow khi counter wrap ~17.2s) */
#define UWB_TS40_MASK   0xFFFFFFFFFFULL

/* ========================================================================== */
/*                     STATE MACHINE                                           */
/* ========================================================================== */

typedef enum {
    ANCHOR_STATE_RX_WAIT    = 0,  /* Listening for POLL from TAG */
    ANCHOR_STATE_TX_RESPOND = 1,  /* Waiting for delayed TX DONE (RESP) */
#if UWB_USE_DS_TWR
    ANCHOR_STATE_WAIT_FINAL = 2,  /* RESP sent, waiting for FINAL from TAG */
    ANCHOR_STATE_TX_REPORT  = 3,  /* REPORT being sent, waiting for TXFRS */
#endif
} AnchorState_t;

/* ========================================================================== */
/*                     PRIVATE DATA                                            */
/* ========================================================================== */

static AnchorState_t        s_state      = ANCHOR_STATE_RX_WAIT;
static uint32_t             s_state_tick = 0;   /* HAL_GetTick() at state entry */
static DW1000_RangingResult s_result     = {0};
static uint8_t              s_tx_seq     = 0;

/* Phase 2 (F-09): số lần delayed-TX bị trễ (HPDWARN). Live Expressions. */
volatile uint32_t anchor_delayed_tx_late_count = 0;

/* FIX-01 verify: predicted RMARKER TX (tính trước khi delayed-TX xảy ra)
 * khớp với TX_TIME thật đọc lại sau TXFRS — dùng để tự kiểm chứng semantics
 * đúng trong lúc bring-up. KHÔNG bắt buộc phải luôn = 0 (vài DTU lệch do
 * quantization là bình thường, xem giải thích trong spec). */
static uint64_t s_predicted_resp_tx_rmarker     = 0;
volatile int32_t  anchor_tx_prediction_error_dtu   = 0;
volatile uint32_t anchor_tx_prediction_error_count = 0;

/* ========================================================================== */
/*                     PRIVATE HELPERS                                         */
/* ========================================================================== */

static uint64_t ts_to_u64(const uint8_t *ts)
{
    uint64_t v = 0;
    v  = (uint64_t)ts[0];
    v |= (uint64_t)ts[1] <<  8;
    v |= (uint64_t)ts[2] << 16;
    v |= (uint64_t)ts[3] << 24;
    v |= (uint64_t)ts[4] << 32;
    return v;
}

/**
 * @brief  Reset to RX_WAIT: force radio off, clear status, restart RX.
 *         Used for error recovery from any state.
 */
static void reset_to_rx_wait(void)
{
    DW1000_ForceRxOff();
    DW1000_ClearAllStatus();
    DW1000_StartRx();
    s_state      = ANCHOR_STATE_RX_WAIT;
    s_state_tick = HAL_GetTick();
}

/**
 * @brief  Handle a valid POLL frame: build and schedule delayed RESP TX.
 *         Refactored out of RX_WAIT (Phase 4 §4.5) so WAIT_FINAL can also
 *         call it when TAG starts a new cycle (mixed SS/DS deployment safety).
 *         Moves code as-is — no logic changes inside.
 */
static void handle_poll(const uint8_t *rx_buf)
{
    /* --- Capture POLL RX timestamp ------------------------------------ */
    DW1000_ReadRxTimestamp(s_result.poll_rx_ts);

    /* --- Build RESPONSE frame ---------------------------------------- */
    uint8_t resp[RESP_FRAME_LEN];
    resp[0] = 0x41;
    resp[1] = 0x88;
    resp[2] = s_tx_seq++;
    resp[3] = DW_PAN_ID & 0xFF;
    resp[4] = (DW_PAN_ID >> 8) & 0xFF;
    resp[5] = rx_buf[7];                 /* Dst = TAG's src addr LSB */
    resp[6] = rx_buf[8];                 /* Dst = TAG's src addr MSB */
    resp[7] = ANCHOR_ADDR & 0xFF;        /* Src = this Anchor LSB    */
    resp[8] = (ANCHOR_ADDR >> 8) & 0xFF; /* Src = this Anchor MSB    */
    resp[9] = FRAME_RESP_FUNC;

    /* --- Compute scheduled TX time + predicted RMARKER --------------- *
     * resp_tx          = thời điểm LỆNH phát (ghi vào DX_TIME),        *
     *                    align 9-bit — GIỮ NGUYÊN công thức cũ,        *
     *                    mask 0xFFFFFFFE00 đã tự an toàn 40-bit.        *
     * resp_tx_rmarker  = thời điểm RMARKER THẬT rời anten, dự đoán     *
     *                    bằng cách cộng antenna delay đang active        *
     *                    (FIX-01 — trước đây thiếu bước này).           *
     * t_reply          = POLL-RX-RMARKER → RESP-TX-RMARKER,            *
     *                    mask 40-bit SAU KHI trừ để an toàn qua         *
     *                    điểm wrap ~17.2s của timestamp counter.       */
    uint64_t poll_rx         = ts_to_u64(s_result.poll_rx_ts);
    uint64_t resp_tx         = (poll_rx + REPLY_DELAY_TICKS) & 0xFFFFFFFE00ULL;
    uint64_t resp_tx_rmarker = (resp_tx + UWB_ACTIVE_TX_ANT_DLY) & UWB_TS40_MASK;
    uint64_t t_reply_40      = (resp_tx_rmarker - poll_rx) & UWB_TS40_MASK;
    uint32_t t_reply         = (uint32_t)t_reply_40;

    s_predicted_resp_tx_rmarker = resp_tx_rmarker;   /* lưu để verify ở TX_RESPOND */

    resp[10] = (uint8_t)(t_reply & 0xFF);
    resp[11] = (uint8_t)((t_reply >>  8) & 0xFF);
    resp[12] = (uint8_t)((t_reply >> 16) & 0xFF);
    resp[13] = (uint8_t)((t_reply >> 24) & 0xFF);

    /* --- Program delayed TX time ------------------------------------- */
    uint8_t dx[5];
    dx[0] = (uint8_t)((resp_tx >>  0) & 0xFF);
    dx[1] = (uint8_t)((resp_tx >>  8) & 0xFF);
    dx[2] = (uint8_t)((resp_tx >> 16) & 0xFF);
    dx[3] = (uint8_t)((resp_tx >> 24) & 0xFF);
    dx[4] = (uint8_t)((resp_tx >> 32) & 0xFF);

    DW1000_ClearAllStatus();
    DW1000_SetDelayedTxTime(dx);
    DW1000_WriteTxData(resp, RESP_FRAME_LEN);
    DW1000_SetTxFrameCtrl(RESP_FRAME_LEN + 2);  /* +2 HW FCS */

    if (DW1000_StartTxDelayed() != 0)
    {
        /* F-09: thời điểm phát đã trôi qua (HPDWARN) → TX sẽ không xảy ra.
         * Recovery NGAY thay vì chờ tới ANCHOR_TX_TIMEOUT (mất nhiều chu kỳ). */
        anchor_delayed_tx_late_count++;
        reset_to_rx_wait();
        return;
    }

    /* Transition to TX_RESPOND, record tick for timeout guard */
    s_state      = ANCHOR_STATE_TX_RESPOND;
    s_state_tick = HAL_GetTick();
}

/* ========================================================================== */
/*                     PUBLIC API                                              */
/* ========================================================================== */

int Anchor_Init(void)
{
    if (DW1000_Init() != 0)
        return -1;

    DW1000_Configure();                     /* Also writes SYS_MASK */
    DW1000_SetAddress(DW_PAN_ID, ANCHOR_ADDR);
    if (DW1000_EnableFastSPI() == 0U)
        return -2;
    DW1000_ClearAllStatus();               /* Ensure IRQ pin is LOW */
    memset(&s_result, 0, sizeof(s_result));
    return 0;
}

void Anchor_StartListening(void)
{
    DW1000_ClearAllStatus();
    DW1000_StartRx();
    s_state      = ANCHOR_STATE_RX_WAIT;
    s_state_tick = HAL_GetTick();
}

void Anchor_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* Không có IRQ: lúc này mới được phép xét timeout/recovery. */
    if (!dw1000_irq_flag)
    {
        if (s_state == ANCHOR_STATE_TX_RESPOND)
        {
            if ((now - s_state_tick) > ANCHOR_TX_TIMEOUT_MS)
            {
                s_result.timeout_count++;
                reset_to_rx_wait();
            }
        }
        else if (s_state == ANCHOR_STATE_RX_WAIT)
        {
            /* If we stay in RX for >200ms without receiving anything,
             * the DW1000 receiver might be locked up (known silicon issue).
             * Restart RX to recover. */
            if ((now - s_state_tick) > 200U)
            {
                reset_to_rx_wait();
            }
        }
#if UWB_USE_DS_TWR
        else if (s_state == ANCHOR_STATE_WAIT_FINAL)
        {
            /* FINAL must arrive within ANCHOR_FINAL_TIMEOUT_MS.
             * If TAG is SS-only, it will never send FINAL → timeout → return to RX_WAIT. */
            if ((now - s_state_tick) > ANCHOR_FINAL_TIMEOUT_MS)
            {
                reset_to_rx_wait();
            }
        }
        else if (s_state == ANCHOR_STATE_TX_REPORT)
        {
            /* TX_REPORT timeout — reuse ANCHOR_TX_TIMEOUT_MS */
            if ((now - s_state_tick) > ANCHOR_TX_TIMEOUT_MS)
            {
                s_result.timeout_count++;
                reset_to_rx_wait();
            }
        }
#endif
        return;
    }

    /* Có IRQ: xử lý trước mọi timeout. */
    dw1000_irq_flag = 0;

    uint32_t status = DW1000_ReadStatus();

    /* ================================================================== */
    switch (s_state)
    {
        /* ============================================================== */
        case ANCHOR_STATE_RX_WAIT:
        /* ============================================================== */
        {
            /* --- Check for RX error ------------------------------------ */
            if ((status & DW_ALL_RX_GOOD) != DW_ALL_RX_GOOD)
            {
                /* FCS error or other RX fault — restart receiver */
                s_result.timeout_count++;
                reset_to_rx_wait();
                break;
            }

            /* --- Read received frame ----------------------------------- */
            uint8_t  rx_buf[MAX_RX_FRAME_LEN];
            uint16_t rx_len = DW1000_ReadRxData(rx_buf, MAX_RX_FRAME_LEN);

            /* Must be at least 10 bytes: header */
            if (rx_len < 10)
            {
                reset_to_rx_wait();
                break;
            }

            /* Must be a POLL frame */
            if (rx_buf[9] != FRAME_POLL_FUNC)
            {
                /* Stray frame (e.g. another Anchor's RESP) — ignore */
                DW1000_ClearAllStatus();
                DW1000_StartRx();
                break;
            }

            /* Must be addressed to THIS Anchor */
            uint16_t dst_addr = rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
            if (dst_addr != ANCHOR_ADDR)
            {
                /* POLL for another anchor — ignore silently */
                DW1000_ClearAllStatus();
                DW1000_StartRx();
                break;
            }

            /* --- Valid POLL received ----------------------------------- */
            handle_poll(rx_buf);
            break;
        }

        /* ============================================================== */
        case ANCHOR_STATE_TX_RESPOND:
        /* ============================================================== */
        {
            if (status & DW_TXFRS_BIT)
            {
                /* TX completed successfully — read back actual TX timestamp */
                DW1000_ReadTxTimestamp(s_result.resp_tx_ts);
                s_result.ranging_count++;

                /* FIX-01 verify: so predicted RMARKER (tính trước lúc lên lịch)
                 * với TX_TIME thật đọc được — xác nhận semantics đúng khi bring-up.
                 * Sai số vài DTU là bình thường do quantization của DW1000. */
                uint64_t actual = ts_to_u64(s_result.resp_tx_ts);
                int64_t  err40  = (int64_t)((actual - s_predicted_resp_tx_rmarker) & UWB_TS40_MASK);
                if (err40 & (1LL << 39))        /* sign-extend 40-bit → 64-bit */
                    err40 -= (1LL << 40);
                anchor_tx_prediction_error_dtu = (int32_t)err40;
                if (err40 < -5 || err40 > 5)    /* tolerance: ±5 DTU */
                    anchor_tx_prediction_error_count++;

#if UWB_USE_DS_TWR
                /* DS-TWR: instead of returning to RX_WAIT, stay and wait for FINAL */
                DW1000_ClearAllStatus();
                DW1000_StartRx();
                s_state      = ANCHOR_STATE_WAIT_FINAL;
                s_state_tick = HAL_GetTick();
#else
                /* SS-TWR (default): LED blink here = 1 SS exchange complete */
                LED_TOGGLE();
                reset_to_rx_wait();
#endif
            }
            else
            {
                /* Unexpected IRQ in TX_RESPOND state — recover */
                reset_to_rx_wait();
            }
            break;
        }

#if UWB_USE_DS_TWR
        /* ============================================================== */
        case ANCHOR_STATE_WAIT_FINAL:
        /* ============================================================== */
        {
            /* FIX-03 pattern: process IRQ first; timeout handled by guard above */

            if ((status & DW_ALL_RX_GOOD) != DW_ALL_RX_GOOD)
            {
                /* RX error in WAIT_FINAL — back to RX_WAIT, no DS this cycle */
                reset_to_rx_wait();
                break;
            }

            uint8_t  rx_buf[MAX_RX_FRAME_LEN];
            uint16_t rx_len = DW1000_ReadRxData(rx_buf, MAX_RX_FRAME_LEN);

            if (rx_len < 10)
            {
                /* Frame too short — stray, keep waiting (GIỮ deadline — FIX-03) */
                DW1000_ClearAllStatus();
                DW1000_StartRx();
                break;
            }

            uint16_t dst_addr = rx_buf[5] | ((uint16_t)rx_buf[6] << 8);
            uint16_t src_addr = rx_buf[7] | ((uint16_t)rx_buf[8] << 8);

            if (rx_buf[9] == FRAME_FINAL_FUNC
                && dst_addr == ANCHOR_ADDR
                && src_addr == TAG_ADDR)
            {
                /* Valid FINAL received — compute Rb = T6 - T3_real */
                uint8_t final_rx_ts[5];
                DW1000_ReadRxTimestamp(final_rx_ts);   /* T6 */

                uint64_t t6 = ts_to_u64(final_rx_ts);
                uint64_t t3 = ts_to_u64(s_result.resp_tx_ts);  /* T3 THẬT (TX_TIME đọc lại) */
                uint32_t rb = (uint32_t)((t6 - t3) & UWB_TS40_MASK);

                /* Build REPORT frame: 14 bytes, func=FRAME_REPORT_FUNC, payload=rb */
                uint8_t report[REPORT_FRAME_LEN];
                report[0] = 0x41;
                report[1] = 0x88;
                report[2] = s_tx_seq++;
                report[3] = DW_PAN_ID & 0xFF;
                report[4] = (DW_PAN_ID >> 8) & 0xFF;
                report[5] = (uint8_t)(src_addr & 0xFF);        /* Dst = TAG LSB */
                report[6] = (uint8_t)((src_addr >> 8) & 0xFF); /* Dst = TAG MSB */
                report[7] = ANCHOR_ADDR & 0xFF;                 /* Src = me LSB */
                report[8] = (ANCHOR_ADDR >> 8) & 0xFF;          /* Src = me MSB */
                report[9] = FRAME_REPORT_FUNC;
                report[10] = (uint8_t)(rb & 0xFF);
                report[11] = (uint8_t)((rb >>  8) & 0xFF);
                report[12] = (uint8_t)((rb >> 16) & 0xFF);
                report[13] = (uint8_t)((rb >> 24) & 0xFF);

                /* Immediate TX — no HPDWARN possible here */
                DW1000_ClearAllStatus();
                DW1000_WriteTxData(report, REPORT_FRAME_LEN);
                DW1000_SetTxFrameCtrl(REPORT_FRAME_LEN + 2);  /* +2 HW FCS */
                DW1000_StartTx();
                s_state      = ANCHOR_STATE_TX_REPORT;
                s_state_tick = HAL_GetTick();
            }
            else if (rx_buf[9] == FRAME_POLL_FUNC && dst_addr == ANCHOR_ADDR)
            {
                /* TAG started a new cycle (SS-only TAG, or FINAL was lost).
                 * Abort DS in progress, handle new POLL normally.
                 * This is the safety choke for mixed SS/DS deployment. */
                handle_poll(rx_buf);
            }
            else
            {
                /* Stray frame — keep waiting, DO NOT reset deadline (FIX-03) */
                DW1000_ClearAllStatus();
                DW1000_StartRx();
            }
            break;
        }

        /* ============================================================== */
        case ANCHOR_STATE_TX_REPORT:
        /* ============================================================== */
        {
            if (status & DW_TXFRS_BIT)
            {
                /* REPORT sent successfully — blink LED = 1 DS exchange complete */
                LED_TOGGLE();
            }
            /* Whether TX success or unexpected IRQ, always return to RX_WAIT.
             * Do NOT run FIX-01 verify block here — that's only for delayed TX (RESP).
             * REPORT is immediate TX; there is no prediction to verify. */
            reset_to_rx_wait();
            break;
        }
#endif /* UWB_USE_DS_TWR */

        /* ============================================================== */
        default:
        /* ============================================================== */
            reset_to_rx_wait();
            break;
    }
}

const DW1000_RangingResult* Anchor_GetResult(void)
{
    return &s_result;
}
