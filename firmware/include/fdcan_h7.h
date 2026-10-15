#ifndef FDCAN_H7_H
#define FDCAN_H7_H

// FDCAN1 on the Nucleo-H753ZI, classic CAN at 1 Mbps, polled (no IRQs).
//   PD0 = FDCAN1_RX = CN9 pin 25  <- transceiver RXD/CRX
//   PD1 = FDCAN1_TX = CN9 pin 27  -> transceiver TXD/CTX
// Same CAN_message_t / read() / write() shape as FlexCAN_T4, so ak10_mit.h's
// motor code is shared with the Teensy build.
//
// Bit timing: kernel clock = HSE 8 MHz (ST-LINK MCO in bypass; the stm32duino
// NUCLEO_H753ZI SystemClock_Config switches it on). Prescaler 1, 1+6+1 = 8 tq
// -> 1 Mbps, 87.5 % sample point. That is NBTP = 0x500, the value verified on
// this board (HARDWARE.md). Needs -DHAL_FDCAN_MODULE_ENABLED (platformio.ini).

#include <Arduino.h>

struct CAN_message_t
{
    uint32_t id = 0;
    uint8_t len = 8;
    uint8_t buf[8] = {0};
    struct { bool extended = false; } flags;
};

class FdcanH7
{
public:
    bool begin()
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
        GPIO_InitTypeDef gpio = {};
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &gpio);

        __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_HSE);
        __HAL_RCC_FDCAN_CLK_ENABLE();

        h_.Instance = FDCAN1;
        h_.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
        h_.Init.Mode = FDCAN_MODE_NORMAL;
        h_.Init.AutoRetransmission = ENABLE;
        h_.Init.TransmitPause = DISABLE;
        h_.Init.ProtocolException = DISABLE;
        h_.Init.NominalPrescaler = 1;
        h_.Init.NominalSyncJumpWidth = 1;
        h_.Init.NominalTimeSeg1 = 6;
        h_.Init.NominalTimeSeg2 = 1;
        h_.Init.DataPrescaler = 1; // data phase unused in classic mode
        h_.Init.DataSyncJumpWidth = 1;
        h_.Init.DataTimeSeg1 = 1;
        h_.Init.DataTimeSeg2 = 1;
        h_.Init.MessageRAMOffset = 0;
        h_.Init.StdFiltersNbr = 0;
        h_.Init.ExtFiltersNbr = 0;
        h_.Init.RxFifo0ElmtsNbr = 64;
        h_.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
        h_.Init.RxFifo1ElmtsNbr = 0;
        h_.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
        h_.Init.RxBuffersNbr = 0;
        h_.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
        h_.Init.TxEventsNbr = 0;
        h_.Init.TxBuffersNbr = 0;
        h_.Init.TxFifoQueueElmtsNbr = 16;
        h_.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
        h_.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

        // No filters: every frame lands in FIFO0. Overwrite mode keeps the
        // newest feedback if the loop stalls (e.g. a long serial write).
        return HAL_FDCAN_Init(&h_) == HAL_OK &&
               HAL_FDCAN_ConfigGlobalFilter(&h_, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK &&
               HAL_FDCAN_ConfigRxFifoOverwrite(&h_, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) == HAL_OK &&
               HAL_FDCAN_Start(&h_) == HAL_OK;
    }

    bool read(CAN_message_t &msg)
    {
        // Bus-off latches CCCR.INIT and the M_CAN never leaves it on its own
        // (the Teensy's FlexCAN auto-recovers). Without this a bus glitch
        // leaves the motors running their last MIT command until a reset.
        if (FDCAN1->PSR & FDCAN_PSR_BO)
            FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;

        FDCAN_RxHeaderTypeDef rx;
        if (HAL_FDCAN_GetRxFifoFillLevel(&h_, FDCAN_RX_FIFO0) == 0 ||
            HAL_FDCAN_GetRxMessage(&h_, FDCAN_RX_FIFO0, &rx, msg.buf) != HAL_OK)
            return false;
        msg.id = rx.Identifier;
        msg.flags.extended = rx.IdType == FDCAN_EXTENDED_ID;
        uint32_t dlc = rx.DataLength;
        if (dlc > 0xF)
            dlc >>= 16; // HAL releases before 1.11 keep the DLC in bits 19:16
        msg.len = dlc > 8 ? 8 : dlc;
        return true;
    }

    // ak10 frames are always 8 bytes, so msg.len is not consulted.
    bool write(const CAN_message_t &msg)
    {
        FDCAN_TxHeaderTypeDef tx = {};
        tx.Identifier = msg.id;
        tx.IdType = msg.flags.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        tx.TxFrameType = FDCAN_DATA_FRAME;
        tx.DataLength = FDCAN_DLC_BYTES_8;
        tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx.BitRateSwitch = FDCAN_BRS_OFF;
        tx.FDFormat = FDCAN_CLASSIC_CAN;
        tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx.MessageMarker = 0;
        return HAL_FDCAN_AddMessageToTxFifoQ(&h_, &tx, (uint8_t *)msg.buf) == HAL_OK;
    }

private:
    FDCAN_HandleTypeDef h_ = {};
};

#endif
