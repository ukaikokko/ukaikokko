#ifndef __UKAIKOKKO_I2C_BUS_H
#define __UKAIKOKKO_I2C_BUS_H

#include <main.h>

namespace ukaikokko
{

class I2C_BUS
{
   public:
    I2C_BUS(I2C_HandleTypeDef* hi2c) : _hi2c(hi2c) {}
    ~I2C_BUS() = default;
    HAL_StatusTypeDef start() const;
    HAL_StatusTypeDef stop() const;
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef isDeviceReady(uint16_t DevAddress, uint32_t Trials = 1, uint32_t Timeout = 10) const;
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef write(uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY) const;
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef read(uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY) const;
    /// 8bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memWrite8(uint16_t DevAddress, uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY) const;
    /// 16bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memWrite16(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY) const;
    /// 8bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memRead8(uint16_t DevAddress, uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY) const;
    /// 16bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memRead16(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY) const;

   private:
    I2C_HandleTypeDef* _hi2c;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_I2C_BUS_H
