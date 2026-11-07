#ifndef __UKAIKOKKO_RING_BUFFER_H
#define __UKAIKOKKO_RING_BUFFER_H

#include <main.h>

namespace ukaikokko
{

template <typename T, size_t BUF_SIZE>
class RingBuffer
{
    static_assert(BUF_SIZE >= 2);

   public:
    HAL_StatusTypeDef get(T& data);
    HAL_StatusTypeDef put(const T& data);
    T* nextPutAddress();
    void headMove();
    bool isOverFlow() { return _overFlow; }

   private:
    volatile size_t _head = 0;
    volatile size_t _tail = 0;
    volatile bool _overFlow = false;
    bool _trashCanFlag = true;
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
    _tail = (_tail + 1) % BUF_SIZE;
    return HAL_OK;
}

template <typename T, size_t BUF_SIZE>
HAL_StatusTypeDef RingBuffer<T, BUF_SIZE>::put(const T& data)
{
    const size_t nextHead = (_head + 1) % BUF_SIZE;
    if (nextHead == _tail)
    {
        _overFlow = true;
        return HAL_ERROR;
    }
    _buf[_head] = data;
    _head = nextHead;
    return HAL_OK;
}

template <typename T, size_t BUF_SIZE>
T* RingBuffer<T, BUF_SIZE>::nextPutAddress()
{
    const size_t nextHead = (_head + 1) % BUF_SIZE;
    if (nextHead == _tail)
    {
        _overFlow = true;
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
        _head = (_head + 1) % BUF_SIZE;
    }
}

} // namespace ukaikokko

#endif // __UKAIKOKKO_RING_BUFFER_H
