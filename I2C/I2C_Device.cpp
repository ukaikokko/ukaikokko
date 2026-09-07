#include "I2C_Device.h"

namespace ukaikokko
{

void I2C_Device::setTimeout(uint32_t timeout)
{
    _timeout = timeout;
}

HAL_StatusTypeDef I2C_Device::isDeviceReady(uint32_t Trials, uint32_t Timeout)
{
    return _bus.isDeviceReady(_address, Trials, Timeout);
}

HAL_StatusTypeDef I2C_Device::write(uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return _bus.write(_address, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Device::read(uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return _bus.read(_address, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Device::memWrite8(uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return _bus.memWrite8(_address, MemAddress, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Device::memWrite16(uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return _bus.memWrite16(_address, MemAddress, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Device::memRead8(uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return _bus.memRead8(_address, MemAddress, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_Device::memRead16(uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return _bus.memRead16(_address, MemAddress, pData, Size, Timeout);
}

} // namespace ukaikokko
