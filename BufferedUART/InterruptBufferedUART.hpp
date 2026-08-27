#ifndef __INTERRUPT_BUFFERED_UART_H
#define __INTERRUPT_BUFFERED_UART_H

#include <main.h>

namespace ukaikokko
{

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
class InterruptBufferedUART
{
    static_assert(RX_BUF_SIZE >= 2);
    static_assert(TX_BUF_SIZE >= 2);

   public:
    InterruptBufferedUART(UART_HandleTypeDef* huart) : _huart(huart) {}
    ~InterruptBufferedUART() = default;
    HAL_StatusTypeDef begin();
    void RxCplt(UART_HandleTypeDef* huart);
    void TxCplt(UART_HandleTypeDef* huart);
    void periodic();
    bool available() const;
    int16_t read();
    bool write(uint8_t data) { return write(&data, 1); }
    bool write(uint8_t* data, size_t len);
    bool getError() const;
    size_t getRxOverflowCount() const { return _rxOverflowCount; }
    size_t getTxInvalidCount() const { return _txInvalidCount; }

   private:
    UART_HandleTypeDef* _huart;

    uint8_t _rxBuf[RX_BUF_SIZE];
    volatile size_t _rxBufHead = 0;
    volatile size_t _rxBufTail = 0;
    uint8_t _txBuf[TX_BUF_SIZE];
    volatile size_t _txBufHead = 0;
    volatile size_t _txBufTail = 0;

    volatile size_t _rxOverflowCount = 0;
    size_t _txInvalidCount = 0;

    volatile bool _txInProgress = false;
};

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
HAL_StatusTypeDef InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::begin()
{
    return HAL_UART_Receive_IT(_huart, &(_rxBuf[_rxBufHead]), 1);
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::RxCplt(UART_HandleTypeDef* huart)
{
    if (huart != _huart) // other UART
    {
        return;
    }

    size_t nextHead = (_rxBufHead + 1) % RX_BUF_SIZE;
    if (nextHead == _rxBufTail)
    {
        // RX buffer overflow
        _rxOverflowCount++;
        HAL_UART_Receive_IT(_huart, &(_rxBuf[_rxBufHead]), 1); // 最も新しいデータを上書き
    }
    else
    {
        _rxBufHead = nextHead;
        HAL_UART_Receive_IT(_huart, &(_rxBuf[_rxBufHead]), 1);
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::TxCplt(UART_HandleTypeDef* huart)
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

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::periodic()
{
    if (!_txInProgress && _txBufHead != _txBufTail) // 割り込みのタイミングによっては送り損ねる可能性があるので，保険として
    {
        _txInProgress = true;
        HAL_UART_Transmit_IT(_huart, &(_txBuf[_txBufTail]), 1);
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::available() const
{
    return (_rxBufHead != _rxBufTail);
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
int16_t InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::read()
{
    if (!available())
    {
        return -1; // No data
    }

    const uint8_t data = _rxBuf[_rxBufTail];
    _rxBufTail = (_rxBufTail + 1) % RX_BUF_SIZE;
    return data;
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::write(uint8_t* data, size_t len)
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

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::getError() const
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
    return isError;
}

} // namespace ukaikokko

#endif // __INTERRUPT_BUFFERED_UART_H
