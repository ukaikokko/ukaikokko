#ifndef __UKAIKOKKO_I2C_DEVICE_H
#define __UKAIKOKKO_I2C_DEVICE_H

#include <main.h>
#include "I2C_BUS.h"

namespace ukaikokko
{

class I2C_Device
{
   public:
    /// @param address 7bit address
    I2C_Device(I2C_BUS& bus, uint16_t address, uint32_t timeout = HAL_MAX_DELAY) : _bus(bus), _address(address), _timeout(timeout) {}
    ~I2C_Device() = default;

    void setTimeout(uint32_t timeout);

    HAL_StatusTypeDef isDeviceReady(uint32_t Trials = 1) const { return isDeviceReady(Trials, _timeout); }
    HAL_StatusTypeDef isDeviceReady(uint32_t Trials, uint32_t Timeout) const;

    HAL_StatusTypeDef write(uint8_t* pData, uint16_t Size) const { return write(pData, Size, _timeout); }
    HAL_StatusTypeDef write(uint8_t* pData, uint16_t Size, uint32_t Timeout) const;

    HAL_StatusTypeDef read(uint8_t* pData, uint16_t Size) const { return read(pData, Size, _timeout); }
    HAL_StatusTypeDef read(uint8_t* pData, uint16_t Size, uint32_t Timeout) const;

    /// 8bit memory address
    HAL_StatusTypeDef memWrite8(uint8_t MemAddress, uint8_t* pData, uint16_t Size) const { return memWrite8(MemAddress, pData, Size, _timeout); }
    /// 8bit memory address
    HAL_StatusTypeDef memWrite8(uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout) const;

    /// 16bit memory address
    HAL_StatusTypeDef memWrite16(uint16_t MemAddress, uint8_t* pData, uint16_t Size) const { return memWrite16(MemAddress, pData, Size, _timeout); }
    /// 16bit memory address
    HAL_StatusTypeDef memWrite16(uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout) const;

    /// 8bit memory address
    HAL_StatusTypeDef memRead8(uint8_t MemAddress, uint8_t* pData, uint16_t Size) const { return memRead8(MemAddress, pData, Size, _timeout); }
    /// 8bit memory address
    HAL_StatusTypeDef memRead8(uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout) const;

    /// 16bit memory address
    HAL_StatusTypeDef memRead16(uint16_t MemAddress, uint8_t* pData, uint16_t Size) const { return memRead16(MemAddress, pData, Size, _timeout); }
    /// 16bit memory address
    HAL_StatusTypeDef memRead16(uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout) const;

   private:
    I2C_BUS& _bus;
    uint16_t _address;
    uint32_t _timeout;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_I2C_DEVICE_H
