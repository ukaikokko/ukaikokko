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

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
class BufferedCAN
{
    static_assert(SYS_RX_BUF_SIZE >= 2);
    static_assert(USER_RX_BUF_SIZE >= 2);
    static_assert(TX_BUF_SIZE >= 2);

   public:
    BufferedCAN(CAN_HandleTypeDef* hcan) : _hcan(hcan) {}
    ~BufferedCAN() = default;
    HAL_StatusTypeDef configFilter(CAN_FilterTypeDef* sFilterConfig) const { return HAL_CAN_ConfigFilter(_hcan, sFilterConfig); }
    HAL_StatusTypeDef begin() const { return HAL_CAN_Start(_hcan); }
    HAL_StatusTypeDef activateNotification(uint32_t ActiveITs) const { return HAL_CAN_ActivateNotification(_hcan, ActiveITs); }
    void RxCplt(CAN_HandleTypeDef* hcan, unsigned int fifo);
    void TxCplt(CAN_HandleTypeDef* hcan);
    void periodic();
    size_t available() const;
    bool read(CANMessage* msg);
    bool write(const CANMessage* msg);
    bool getError() const;
    size_t getSysRxOverflowCount() const { return _sysRxOverflowCount; };
    size_t getUserRxOverflowCount() const { return _userRxOverflowCount; };
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
    CompactCANMessage _sysRxBuf[SYS_RX_BUF_SIZE];
    volatile size_t _sysRxBufHead = 0;
    volatile size_t _sysRxBufTail = 0;
    CompactCANMessage _userRxBuf[USER_RX_BUF_SIZE];
    size_t _userRxBufHead = 0;
    size_t _userRxBufTail = 0;
    size_t _userRxBufCount = 0;
    CompactCANMessage _txBuf[TX_BUF_SIZE];
    size_t _txBufHead = 0;
    size_t _txBufTail = 0;

    volatile size_t _sysRxOverflowCount = 0;
    size_t _userRxOverflowCount = 0;
    size_t _txInvalidCount = 0;
    unsigned int _addErrorCount = 0;

    bool _txInProgress = false;
};

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
void BufferedCAN<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::RxCplt(CAN_HandleTypeDef* hcan, unsigned int fifo)
{
    if (hcan != _hcan) // other CAN
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifo) > 0)
    {
        CAN_RxHeaderTypeDef rxHeader;
        if (HAL_CAN_GetRxMessage(_hcan, fifo, &rxHeader, _sysRxBuf[_sysRxBufHead].data) == HAL_OK)
        {
            if (rxHeader.IDE == CAN_ID_STD)
            {
                _sysRxBuf[_sysRxBufHead].id = rxHeader.StdId;
            }
            else
            {
                _sysRxBuf[_sysRxBufHead].id = rxHeader.ExtId;
                _sysRxBuf[_sysRxBufHead].id |= EXT_ID_FLAG; // Extended ID flag
            }
            _sysRxBuf[_sysRxBufHead].dlc = rxHeader.DLC;

            size_t nextHead = (_sysRxBufHead + 1) % SYS_RX_BUF_SIZE;
            if (nextHead == _sysRxBufTail)
            {
                // System RX buffer overflow
                _sysRxOverflowCount++;
            }
            else
            {
                _sysRxBufHead = nextHead;
            }
        }
    }
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
void BufferedCAN<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::TxCplt(CAN_HandleTypeDef* hcan)
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

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
void BufferedCAN<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::periodic()
{
    while (_sysRxBufTail != _sysRxBufHead)
    {
        size_t nextUserHead = (_userRxBufHead + 1) % USER_RX_BUF_SIZE;
        if (nextUserHead == _userRxBufTail)
        {
            // User RX buffer overflow
            _userRxOverflowCount++;
            _sysRxBufTail = (_sysRxBufTail + 1) % SYS_RX_BUF_SIZE; // どんどんデータが捨てられる
        }
        else
        {
            _userRxBuf[_userRxBufHead] = _sysRxBuf[_sysRxBufTail];
            _sysRxBufTail = (_sysRxBufTail + 1) % SYS_RX_BUF_SIZE;
            _userRxBufHead = nextUserHead;
            _userRxBufCount++;
        }
    }

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

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
size_t BufferedCAN<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::available() const
{
    return _userRxBufCount;
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::read(CANMessage* msg)
{
    if (_userRxBufCount == 0)
    {
        return false; // No data
    }

    for (size_t i = 0; i < 8; i++)
    {
        msg->data[i] = _userRxBuf[_userRxBufTail].data[i];
    }
    msg->id = _userRxBuf[_userRxBufTail].id;
    msg->dlc = _userRxBuf[_userRxBufTail].dlc;
    if (msg->id & EXT_ID_FLAG)
    {
        msg->ide = CAN_ID_EXT;   // Extended ID
        msg->id &= ~EXT_ID_FLAG; // Clear Extended ID flag
    }
    else
    {
        msg->ide = CAN_ID_STD; // Standard ID
    }
    _userRxBufTail = (_userRxBufTail + 1) % USER_RX_BUF_SIZE;
    _userRxBufCount--;
    return true;
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::write(const CANMessage* msg)
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

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool BufferedCAN<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::getError() const
{
    bool isError = false;
    if (_sysRxOverflowCount > 0)
    {
        isError = true;
    }
    if (_userRxOverflowCount > 0)
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
