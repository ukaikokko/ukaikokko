#ifndef __UKAIKOKKO_BUFFERED_CAN_H
#define __UKAIKOKKO_BUFFERED_CAN_H

#include <main.h>
#include "../RingBuffer/RingBuffer.hpp"

namespace ukaikokko
{

struct CANMessage
{
    uint32_t id;
    uint32_t ide;
    uint8_t dlc;
    uint8_t data[8];
};

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
class BufferedCAN
{
    static_assert(RX_BUF_SIZE >= 2);
    static_assert((RX_BUF_SIZE & (RX_BUF_SIZE - 1)) == 0); // 2のべき乗のみを許可
    static_assert(TX_BUF_SIZE >= 2);
    static_assert((TX_BUF_SIZE & (TX_BUF_SIZE - 1)) == 0); // 2のべき乗のみを許可

   public:
    BufferedCAN(CAN_HandleTypeDef* hcan) : _hcan(hcan) {}
    ~BufferedCAN() = default;
    HAL_StatusTypeDef configFilter(CAN_FilterTypeDef* sFilterConfig) const { return HAL_CAN_ConfigFilter(_hcan, sFilterConfig); }
    HAL_StatusTypeDef start() const { return HAL_CAN_Start(_hcan); }
    HAL_StatusTypeDef activateNotification(uint32_t ActiveITs) const { return HAL_CAN_ActivateNotification(_hcan, ActiveITs); }
    void RxCplt(CAN_HandleTypeDef* hcan, unsigned int fifo);
    void TxCplt(CAN_HandleTypeDef* hcan);
    void periodic();
    bool available() const;
    bool read(CANMessage* msg);
    bool write(const CANMessage* msg);
    bool getError() const;
    size_t getRxOverflowCount() const { return _rxBuf.overFlowCount(); }
    size_t getTxInvalidCount() const { return _txBuf.overFlowCount(); }
    size_t getAddErrorCount() const { return _addErrorCount; }

   private:
    CAN_HandleTypeDef* _hcan;

    struct CompactCANMessage
    {
        uint32_t id;
        uint8_t dlc;
        uint8_t data[8];
    };
    static constexpr uint32_t EXT_ID_FLAG = 1UL << 31;

    RingBuffer<CompactCANMessage, RX_BUF_SIZE> _rxBuf;
    RingBuffer<CompactCANMessage, TX_BUF_SIZE> _txBuf;
    volatile size_t _addErrorCount = 0;

    volatile bool _txInProgress = false;
};

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::RxCplt(CAN_HandleTypeDef* hcan, unsigned int fifo)
{
    if (hcan != _hcan) // other CAN
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifo) > 0)
    {
        CAN_RxHeaderTypeDef rxHeader;
        CompactCANMessage* msg = _rxBuf.nextPutAddress();
        if (HAL_CAN_GetRxMessage(_hcan, fifo, &rxHeader, msg->data) == HAL_OK)
        {
            if (rxHeader.IDE == CAN_ID_STD)
            {
                msg->id = rxHeader.StdId;
            }
            else
            {
                msg->id = rxHeader.ExtId;
                msg->id |= EXT_ID_FLAG; // Extended ID flag
            }
            msg->dlc = rxHeader.DLC;
            _rxBuf.headMove();
        }
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::TxCplt(CAN_HandleTypeDef* hcan)
{
    if (hcan != _hcan) // other CAN
    {
        return;
    }

    if (_txBuf.empty() && HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 3)
    { // 最後の割り込み
        _txInProgress = false;
        return;
    }

    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan))
    {
        if (_txBuf.empty())
        {
            break;
        }
        CompactCANMessage* msg = _txBuf.nextGetAddress();
        uint32_t txMailbox;
        CAN_TxHeaderTypeDef txHeader;
        if (msg->id & EXT_ID_FLAG)
        {
            txHeader.ExtId = msg->id & ~EXT_ID_FLAG; // Clear Extended ID flag
            txHeader.IDE = CAN_ID_EXT;
        }
        else
        {
            txHeader.StdId = msg->id;
            txHeader.IDE = CAN_ID_STD;
        }
        txHeader.RTR = CAN_RTR_DATA; // data frame
        txHeader.DLC = msg->dlc;
        txHeader.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(_hcan, &txHeader, msg->data, &txMailbox) == HAL_OK)
        {
            _txBuf.tailMove();
        }
        else
        {
            _addErrorCount++;
            if (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 3)
            {
                _txInProgress = false;
            }
            break; // 送信失敗した場合は次のperiodic()か，別のTxCplt()で再送する
        }
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::periodic()
{
    for (int i = 0; i < 10; i++) // Addが失敗する可能性を考慮し何回か
    {
        if (!_txInProgress && !_txBuf.empty()) // 保険
        {
            _txInProgress = true;
            CompactCANMessage* msg = _txBuf.nextGetAddress();
            uint32_t txMailbox;
            CAN_TxHeaderTypeDef txHeader;
            if (msg->id & EXT_ID_FLAG)
            {
                txHeader.ExtId = msg->id & ~EXT_ID_FLAG; // Clear Extended ID flag
                txHeader.IDE = CAN_ID_EXT;
            }
            else
            {
                txHeader.StdId = msg->id;
                txHeader.IDE = CAN_ID_STD;
            }
            txHeader.RTR = CAN_RTR_DATA; // data frame
            txHeader.DLC = msg->dlc;
            txHeader.TransmitGlobalTime = DISABLE;
            if (HAL_CAN_AddTxMessage(_hcan, &txHeader, msg->data, &txMailbox) == HAL_OK)
            {
                _txBuf.tailMove();
            }
            else
            {
                _addErrorCount++;
                _txInProgress = false; // next periodic() will try to send again
            }
        }
        else if (_txInProgress && HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 3)
        {
            // 送信中であるにもかかわらず，3つのTxMailboxが空いている場合は，送信が失敗している可能性があるので，次のperiodic()で再送する
            _txInProgress = false;
        }
        else
        {
            break;
        }
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::available() const
{
    return !(_rxBuf.empty());
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::read(CANMessage* msg)
{
    CompactCANMessage readMsg;
    if (_rxBuf.get(readMsg) != HAL_OK)
    {
        return false; // No data
    }

    for (size_t i = 0; i < 8; i++)
    {
        msg->data[i] = readMsg.data[i];
    }
    msg->id = readMsg.id;
    msg->dlc = readMsg.dlc;
    if (msg->id & EXT_ID_FLAG)
    {
        msg->ide = CAN_ID_EXT;   // Extended ID
        msg->id &= ~EXT_ID_FLAG; // Clear Extended ID flag
    }
    else
    {
        msg->ide = CAN_ID_STD; // Standard ID
    }
    return true;
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::write(const CANMessage* msg)
{
    CompactCANMessage compactMsg;
    if (msg->ide == CAN_ID_EXT)
    {
        compactMsg.id = msg->id | EXT_ID_FLAG; // Set Extended ID flag
    }
    else
    {
        compactMsg.id = msg->id;
    }
    compactMsg.dlc = msg->dlc;
    for (size_t i = 0; i < 8; i++)
    {
        compactMsg.data[i] = msg->data[i];
    }

    if (_txBuf.put(compactMsg) != HAL_OK)
    {
        return false;
    }

    if (!_txInProgress)
    {
        _txInProgress = true;
        CompactCANMessage* msg = _txBuf.nextGetAddress();
        uint32_t txMailbox;
        CAN_TxHeaderTypeDef txHeader;
        if (msg->id & EXT_ID_FLAG)
        {
            txHeader.ExtId = msg->id & ~EXT_ID_FLAG; // Clear Extended ID flag
            txHeader.IDE = CAN_ID_EXT;
        }
        else
        {
            txHeader.StdId = msg->id;
            txHeader.IDE = CAN_ID_STD;
        }
        txHeader.RTR = CAN_RTR_DATA; // data frame
        txHeader.DLC = msg->dlc;
        txHeader.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(_hcan, &txHeader, msg->data, &txMailbox) == HAL_OK)
        {
            _txBuf.tailMove();
        }
        else
        {
            _addErrorCount++;
            _txInProgress = false; // next periodic() will try to send again
        }
    }

    return true;
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::getError() const
{
    bool isError = false;
    if (_rxBuf.overFlowCount() > 0)
    {
        isError = true;
    }
    if (_txBuf.overFlowCount() > 0)
    {
        isError = true;
    }
    if (_addErrorCount > 0)
    {
        isError = true;
    }
    return isError;
}

} // namespace ukaikokko

#endif // __UKAIKOKKO_BUFFERED_CAN_H
