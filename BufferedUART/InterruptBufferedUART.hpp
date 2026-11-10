#ifndef __UKAIKOKKO_INTERRUPT_BUFFERED_UART_H
#define __UKAIKOKKO_INTERRUPT_BUFFERED_UART_H

#include <main.h>
#include "../RingBuffer/RingBuffer.hpp"

namespace ukaikokko
{

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
class InterruptBufferedUART
{
    static_assert(RX_BUF_SIZE >= 2);
    static_assert((RX_BUF_SIZE & (RX_BUF_SIZE - 1)) == 0); // 2のべき乗のみを許可
    static_assert(TX_BUF_SIZE >= 2);
    static_assert((TX_BUF_SIZE & (TX_BUF_SIZE - 1)) == 0); // 2のべき乗のみを許可

   public:
    InterruptBufferedUART(UART_HandleTypeDef* huart) : _huart(huart) {}
    ~InterruptBufferedUART() = default;
    HAL_StatusTypeDef start();
    void RxCplt(UART_HandleTypeDef* huart);
    void TxCplt(UART_HandleTypeDef* huart);
    void whenError(UART_HandleTypeDef* huart);
    void periodic();
    bool available() const;
    int16_t read();
    bool write(uint8_t data) { return write(&data, 1); }
    bool write(uint8_t* data, size_t len);
    bool getError() const;
    size_t getRxOverflowCount() const { return _rxBuf.overFlowCount(); }
    size_t getTxInvalidCount() const { return _txInvalidCount; }
    size_t getTxOverflowCount() const { return _txBuf.overFlowCount(); }

   private:
    UART_HandleTypeDef* _huart;
    RingBuffer<uint8_t, RX_BUF_SIZE> _rxBuf;
    RingBuffer<uint8_t, TX_BUF_SIZE> _txBuf;
    size_t _txInvalidCount = 0;

    volatile bool _txInProgress = false;
};

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
HAL_StatusTypeDef InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::start()
{
    return HAL_UART_Receive_IT(_huart, _rxBuf.nextPutAddress(), 1);
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::RxCplt(UART_HandleTypeDef* huart)
{
    if (huart != _huart) // other UART
    {
        return;
    }

    _rxBuf.headMove();
    HAL_UART_Receive_IT(_huart, _rxBuf.nextPutAddress(), 1);
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::TxCplt(UART_HandleTypeDef* huart)
{
    if (huart != _huart) // other UART
    {
        return;
    }

    _txBuf.tailMove();
    if (!(_txBuf.empty()))
    {
        HAL_UART_Transmit_IT(_huart, _txBuf.nextGetAddress(), 1);
    }
    else
    {
        _txInProgress = false;
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::whenError(UART_HandleTypeDef* huart)
{
    if (huart != _huart) // other UART
    {
        return;
    }
    if (_huart->RxState == HAL_UART_STATE_READY)
    {
        HAL_UART_Receive_IT(_huart, _rxBuf.nextPutAddress(), 1);
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
void InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::periodic()
{
    if (!_txInProgress && !(_txBuf.empty())) // 割り込みのタイミングによっては送り損ねる可能性があるので，保険として
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        // クリティカルセクション開始

        if (!_txInProgress)
        {
            _txInProgress = true;

            // クリティカルセクション終了
            __set_PRIMASK(primask);

            HAL_UART_Transmit_IT(_huart, _txBuf.nextGetAddress(), 1);
        }
        else
        {
            // クリティカルセクション終了
            __set_PRIMASK(primask);
        }
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::available() const
{
    return !(_rxBuf.empty());
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
int16_t InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::read()
{
    uint8_t data = 0;
    if (_rxBuf.get(data) == HAL_OK)
    {
        return data;
    }
    else
    {
        return -1;
    }
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::write(uint8_t* data, size_t len)
{
    if (len == 0)
    {
        return true; // Nothing to write
    }

    if (_txBuf.freeLevel() < len)
    {
        _txInvalidCount++;
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        const uint8_t temp = data[i]; // 先にdata+iの計算を行う

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        // クリティカルセクション開始

        _txBuf.put(temp);

        // クリティカルセクション終了
        __set_PRIMASK(primask);
    }

    if (!_txInProgress)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        // クリティカルセクション開始

        if (!_txInProgress)
        {
            _txInProgress = true;

            // クリティカルセクション終了
            __set_PRIMASK(primask);

            HAL_UART_Transmit_IT(_huart, _txBuf.nextGetAddress(), 1);
        }
        else
        {
            // クリティカルセクション終了
            __set_PRIMASK(primask);
        }
    }

    return true;
}

template <size_t RX_BUF_SIZE, size_t TX_BUF_SIZE>
bool InterruptBufferedUART<RX_BUF_SIZE, TX_BUF_SIZE>::getError() const
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
    if (_txInvalidCount > 0)
    {
        isError = true;
    }
    return isError;
}

} // namespace ukaikokko

#endif // __UKAIKOKKO_INTERRUPT_BUFFERED_UART_H
