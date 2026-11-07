#ifndef __UKAIKOKKO_RING_BUFFER_H
#define __UKAIKOKKO_RING_BUFFER_H

#include <main.h>

namespace ukaikokko
{

/// @tparam BUF_SIZE 2のべき乗である必要があります
template <typename T, size_t BUF_SIZE>
class RingBuffer
{
    static_assert(BUF_SIZE >= 2);
    static_assert((BUF_SIZE & (BUF_SIZE - 1)) == 0); // 2のべき乗のみを許可

   public:
    HAL_StatusTypeDef get(T& data);
    HAL_StatusTypeDef put(const T& data);
    size_t freeLevel() const;
    bool empty() const { return (_head == _tail); }
    size_t overFlowCount() const { return _overFlowCount; }

    /// @brief 実際にgetされた後，tailMove()を呼び出す必要があります
    /// @note 呼び出す前にempty()==falseであることを確認し，getすべきデータが存在することを確認してください
    T* nextGetAddress();
    /// @brief nextGetAddress()からデータが取り出されたのち，呼び出される必要があります
    void tailMove();
    /// @brief 実際にputされた後，headMove()を呼び出す必要があります
    T* nextPutAddress();
    /// @brief nextPutAddress()にデータが格納されたのち，呼び出される必要があります
    void headMove();

   private:
    static constexpr size_t BUF_MASK = BUF_SIZE - 1;
    volatile size_t _head = 0;
    volatile size_t _tail = 0;
    volatile size_t _overFlowCount = 0;
    bool _trashCanFlag = true; // 書き込み側のみでしか使われない
    T _buf[BUF_SIZE];
    T _trashCan;
};

template <typename T, size_t BUF_SIZE>
HAL_StatusTypeDef RingBuffer<T, BUF_SIZE>::get(T& data)
{
    if (_tail == _head)
    {
        return HAL_ERROR;
    }
    data = _buf[_tail];
    _tail = (_tail + 1) & BUF_MASK;
    return HAL_OK;
}

template <typename T, size_t BUF_SIZE>
HAL_StatusTypeDef RingBuffer<T, BUF_SIZE>::put(const T& data)
{
    const size_t nextHead = (_head + 1) & BUF_MASK;
    if (nextHead == _tail)
    {
        _overFlowCount++;
        return HAL_ERROR;
    }
    _buf[_head] = data;
    _head = nextHead;
    return HAL_OK;
}

template <typename T, size_t BUF_SIZE>
size_t RingBuffer<T, BUF_SIZE>::freeLevel() const
{
    if (_head >= _tail)
    {
        return BUF_SIZE - 1 - (_head - _tail);
    }
    else
    {
        return _tail - _head - 1;
    }
}

template <typename T, size_t BUF_SIZE>
T* RingBuffer<T, BUF_SIZE>::nextGetAddress()
{
    return _buf + _tail;
}

template <typename T, size_t BUF_SIZE>
void RingBuffer<T, BUF_SIZE>::tailMove()
{
    _tail = (_tail + 1) & BUF_MASK;
}

template <typename T, size_t BUF_SIZE>
T* RingBuffer<T, BUF_SIZE>::nextPutAddress()
{
    const size_t nextHead = (_head + 1) & BUF_MASK;
    if (nextHead == _tail)
    {
        _overFlowCount++;
        _trashCanFlag = true;
        return &_trashCan;
    }
    _trashCanFlag = false;
    return _buf + _head;
}

template <typename T, size_t BUF_SIZE>
void RingBuffer<T, BUF_SIZE>::headMove()
{
    if (_trashCanFlag == false)
    {
        _head = (_head + 1) & BUF_MASK;
    }
}

} // namespace ukaikokko

#endif // __UKAIKOKKO_RING_BUFFER_H
