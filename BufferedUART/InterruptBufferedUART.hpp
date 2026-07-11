#ifndef __INTERRUPT_BUFFERED_UART_H
#define __INTERRUPT_BUFFERED_UART_H

#include <main.h>

namespace ukaikokko
{

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
class InterruptBufferedUART
{
    static_assert(SYS_RX_BUF_SIZE >= 2);
    static_assert(USER_RX_BUF_SIZE >= 2);
    static_assert(TX_BUF_SIZE >= 2);

   public:
    InterruptBufferedUART(UART_HandleTypeDef* huart) : _huart(huart) {}
    ~InterruptBufferedUART() = default;
    HAL_StatusTypeDef begin();
    void RxCplt(UART_HandleTypeDef* huart);
    void TxCplt(UART_HandleTypeDef* huart);
    void periodic();
    size_t available() const;
    int16_t read();
    bool write(uint8_t data) { return write(&data, 1); }
    bool write(uint8_t* data, size_t len);
    bool getError() const;
    size_t getSysRxOverflowCount() const { return _sysRxOverflowCount; }
    size_t getUserRxOverflowCount() const { return _userRxOverflowCount; }
    size_t getTxInvalidCount() const { return _txInvalidCount; }

   private:
    UART_HandleTypeDef* _huart;

    uint8_t _sysRxBuf[SYS_RX_BUF_SIZE];
    volatile size_t _sysRxBufHead = 0;
    volatile size_t _sysRxBufTail = 0;
    uint8_t _userRxBuf[USER_RX_BUF_SIZE];
    size_t _userRxBufHead = 0;
    size_t _userRxBufTail = 0;
    size_t _userRxBufCount = 0;
    uint8_t _txBuf[TX_BUF_SIZE];
    size_t _txBufHead = 0;
    size_t _txBufTail = 0;

    volatile size_t _sysRxOverflowCount = 0;
    size_t _userRxOverflowCount = 0;
    size_t _txInvalidCount = 0;

    bool _txInProgress = false;
};

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
HAL_StatusTypeDef InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::begin()
{
    return HAL_UART_Receive_IT(_huart, &(_sysRxBuf[_sysRxBufHead]), 1);
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::RxCplt(UART_HandleTypeDef* huart)
{
    if (huart != _huart) // other UART
    {
        return;
    }

    size_t nextHead = (_sysRxBufHead + 1) % SYS_RX_BUF_SIZE;
    if (nextHead == _sysRxBufTail)
    {
        // System RX buffer overflow
        _sysRxOverflowCount++;
        HAL_UART_Receive_IT(_huart, &(_sysRxBuf[_sysRxBufHead]), 1); // 最も新しいデータを上書き
    }
    else
    {
        _sysRxBufHead = nextHead;
        HAL_UART_Receive_IT(_huart, &(_sysRxBuf[_sysRxBufHead]), 1);
    }
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::TxCplt(UART_HandleTypeDef* huart)
{
    if (huart != _huart) // other UART
    {
        return;
    }

    _txBufTail = (_txBufTail + 1) % TX_BUF_SIZE;
    if (_txBufHead == _txBufTail)
    {
        _txInProgress = false;
    }
    else
    {
        HAL_UART_Transmit_IT(_huart, &(_txBuf[_txBufTail]), 1);
    }
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::periodic()
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

    if (!_txInProgress && _txBufHead != _txBufTail) // 割り込みのタイミングによっては送り損ねる可能性があるので，保険として
    {
        _txInProgress = true;
        HAL_UART_Transmit_IT(_huart, &(_txBuf[_txBufTail]), 1);
    }
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
size_t InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::available() const
{
    return _userRxBufCount;
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
int16_t InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::read()
{
    if (_userRxBufCount == 0)
    {
        return -1; // No data
    }

    const uint8_t data = _userRxBuf[_userRxBufTail];
    _userRxBufTail = (_userRxBufTail + 1) % USER_RX_BUF_SIZE;
    _userRxBufCount--;
    return data;
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::write(uint8_t* data, size_t len)
{
    if (len == 0)
    {
        return true; // Nothing to write
    }

    size_t checkHead = _txBufHead;
    for (size_t i = 0; i < len + 1; i++)
    {
        checkHead = (checkHead + 1) % TX_BUF_SIZE;
        if (checkHead == _txBufTail) // ぴったりの場合？→len+1まで確認されるので，head==tailとなる
        {
            // TX buffer overflow
            _txInvalidCount++;
            return false;
        }
    }
    for (size_t i = 0; i < len; i++)
    {
        size_t nextHead = (_txBufHead + 1) % TX_BUF_SIZE;
        _txBuf[_txBufHead] = data[i];
        _txBufHead = nextHead;
    }

    if (!_txInProgress)
    {
        _txInProgress = true;
        HAL_UART_Transmit_IT(_huart, &(_txBuf[_txBufTail]), 1);
    }

    return true;
}

template <size_t SYS_RX_BUF_SIZE, size_t USER_RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<SYS_RX_BUF_SIZE, USER_RX_BUF_SIZE, TX_BUF_SIZE>::getError() const
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
    return isError;
}

} // namespace ukaikokko

#endif // __INTERRUPT_BUFFERED_UART_H
