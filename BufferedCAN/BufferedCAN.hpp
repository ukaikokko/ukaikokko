#ifndef __UKAIKOKKO_BUFFERED_CAN_H
#define __UKAIKOKKO_BUFFERED_CAN_H

#include <main.h>

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
    static_assert(TX_BUF_SIZE >= 2);

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
    size_t getRxOverflowCount() const { return _rxOverflowCount; };
    size_t getTxInvalidCount() const { return _txInvalidCount; };
    unsigned int getAddErrorCount() const { return _addErrorCount; };

   private:
    CAN_HandleTypeDef* _hcan;

    struct CompactCANMessage
    {
        uint32_t id;
        uint8_t dlc;
        uint8_t data[8];
    };
    constexpr static uint32_t EXT_ID_FLAG = 1UL << 31;
    CompactCANMessage _rxBuf[RX_BUF_SIZE];
    volatile size_t _rxBufHead = 0;
    volatile size_t _rxBufTail = 0;
    CompactCANMessage _txBuf[TX_BUF_SIZE];
    volatile size_t _txBufHead = 0;
    volatile size_t _txBufTail = 0;

    volatile size_t _rxOverflowCount = 0;
    size_t _txInvalidCount = 0;
    volatile unsigned int _addErrorCount = 0;

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
        if (HAL_CAN_GetRxMessage(_hcan, fifo, &rxHeader, _rxBuf[_rxBufHead].data) == HAL_OK)
        {
            if (rxHeader.IDE == CAN_ID_STD)
            {
                _rxBuf[_rxBufHead].id = rxHeader.StdId;
            }
            else
            {
                _rxBuf[_rxBufHead].id = rxHeader.ExtId;
                _rxBuf[_rxBufHead].id |= EXT_ID_FLAG; // Extended ID flag
            }
            _rxBuf[_rxBufHead].dlc = rxHeader.DLC;

            size_t nextHead = (_rxBufHead + 1) % RX_BUF_SIZE;
            if (nextHead == _rxBufTail)
            {
                // RX buffer overflow
                _rxOverflowCount++;
            }
            else
            {
                _rxBufHead = nextHead;
            }
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

    if (_txBufHead == _txBufTail && HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 3)
    { // 最後の割り込み
        _txInProgress = false;
    }
    else
    {
        while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) > 0)
        {
            if (_txBufHead == _txBufTail)
            {
                break;
            }
            uint32_t txMailbox;
            CAN_TxHeaderTypeDef txHeader;
            if (_txBuf[_txBufTail].id & EXT_ID_FLAG)
            {
                txHeader.ExtId = _txBuf[_txBufTail].id & ~EXT_ID_FLAG; // Clear Extended ID flag
                txHeader.IDE = CAN_ID_EXT;
            }
            else
            {
                txHeader.StdId = _txBuf[_txBufTail].id;
                txHeader.IDE = CAN_ID_STD;
            }
            txHeader.RTR = CAN_RTR_DATA; // data frame
            txHeader.DLC = _txBuf[_txBufTail].dlc;
            txHeader.TransmitGlobalTime = DISABLE;
            if (HAL_CAN_AddTxMessage(_hcan, &txHeader, _txBuf[_txBufTail].data, &txMailbox) == HAL_OK)
            {
                _txBufTail = (_txBufTail + 1) % TX_BUF_SIZE;
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
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::periodic()
{
    for (int i = 0; i < 10; i++) // Addが失敗する可能性を考慮し何回か
    {
        if (!_txInProgress && _txBufHead != _txBufTail) // 保険
        {
            _txInProgress = true;
            uint32_t txMailbox;
            CAN_TxHeaderTypeDef txHeader;
            if (_txBuf[_txBufTail].id & EXT_ID_FLAG)
            {
                txHeader.ExtId = _txBuf[_txBufTail].id & ~EXT_ID_FLAG; // Clear Extended ID flag
                txHeader.IDE = CAN_ID_EXT;
            }
            else
            {
                txHeader.StdId = _txBuf[_txBufTail].id;
                txHeader.IDE = CAN_ID_STD;
            }
            txHeader.RTR = CAN_RTR_DATA; // data frame
            txHeader.DLC = _txBuf[_txBufTail].dlc;
            txHeader.TransmitGlobalTime = DISABLE;
            if (HAL_CAN_AddTxMessage(_hcan, &txHeader, _txBuf[_txBufTail].data, &txMailbox) == HAL_OK)
            {
                _txBufTail = (_txBufTail + 1) % TX_BUF_SIZE;
            }
            else
            {
                _addErrorCount++;
                _txInProgress = false; // next periodic() will try to send again
            }
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
    return (_rxBufHead != _rxBufTail);
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::read(CANMessage* msg)
{
    if (!available())
    {
        return false; // No data
    }

    for (size_t i = 0; i < 8; i++)
    {
        msg->data[i] = _rxBuf[_rxBufTail].data[i];
    }
    msg->id = _rxBuf[_rxBufTail].id;
    msg->dlc = _rxBuf[_rxBufTail].dlc;
    if (msg->id & EXT_ID_FLAG)
    {
        msg->ide = CAN_ID_EXT;   // Extended ID
        msg->id &= ~EXT_ID_FLAG; // Clear Extended ID flag
    }
    else
    {
        msg->ide = CAN_ID_STD; // Standard ID
    }
    _rxBufTail = (_rxBufTail + 1) % RX_BUF_SIZE;
    return true;
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<RX_BUF_SIZE, TX_BUF_SIZE>::write(const CANMessage* msg)
{
    size_t nextHead = (_txBufHead + 1) % TX_BUF_SIZE;
    if (nextHead == _txBufTail)
    {
        // TX buffer overflow
        _txInvalidCount++;
        return false;
    }

    if (msg->ide == CAN_ID_EXT)
    {
        _txBuf[_txBufHead].id = msg->id | EXT_ID_FLAG; // Set Extended ID flag
    }
    else
    {
        _txBuf[_txBufHead].id = msg->id;
    }
    _txBuf[_txBufHead].dlc = msg->dlc;
    for (size_t i = 0; i < 8; i++)
    {
        _txBuf[_txBufHead].data[i] = msg->data[i];
    }
    _txBufHead = nextHead;

    if (!_txInProgress)
    {
        _txInProgress = true;
        uint32_t txMailbox;
        CAN_TxHeaderTypeDef txHeader;
        if (_txBuf[_txBufTail].id & EXT_ID_FLAG)
        {
            txHeader.ExtId = _txBuf[_txBufTail].id & ~EXT_ID_FLAG; // Clear Extended ID flag
            txHeader.IDE = CAN_ID_EXT;
        }
        else
        {
            txHeader.StdId = _txBuf[_txBufTail].id;
            txHeader.IDE = CAN_ID_STD;
        }
        txHeader.RTR = CAN_RTR_DATA; // data frame
        txHeader.DLC = _txBuf[_txBufTail].dlc;
        txHeader.TransmitGlobalTime = DISABLE;
        if (HAL_CAN_AddTxMessage(_hcan, &txHeader, _txBuf[_txBufTail].data, &txMailbox) == HAL_OK)
        {
            _txBufTail = (_txBufTail + 1) % TX_BUF_SIZE;
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
    if (_rxOverflowCount > 0)
    {
        isError = true;
    }
    if (_txInvalidCount > 0)
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
