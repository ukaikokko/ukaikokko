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
    HAL_StatusTypeDef start();
    HAL_StatusTypeDef stop();
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef isDeviceReady(uint16_t DevAddress, uint32_t Trials = 1, uint32_t Timeout = 10);
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef write(uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY);
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef read(uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY);
    /// 8bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memWrite8(uint16_t DevAddress, uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY);
    /// 16bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memWrite16(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY);
    /// 8bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memRead8(uint16_t DevAddress, uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY);
    /// 16bit memory address
    /// @param DevAddress: 7bit address
    HAL_StatusTypeDef memRead16(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout = HAL_MAX_DELAY);

   private:
    I2C_HandleTypeDef* _hi2c;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_I2C_BUS_H
