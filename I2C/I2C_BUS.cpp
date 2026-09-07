#include "I2C_BUS.h"

namespace ukaikokko
{

HAL_StatusTypeDef I2C_BUS::start()
{
    return HAL_I2C_Init(_hi2c);
}

HAL_StatusTypeDef I2C_BUS::stop()
{
    return HAL_I2C_DeInit(_hi2c);
}

HAL_StatusTypeDef I2C_BUS::isDeviceReady(uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
    return HAL_I2C_IsDeviceReady(_hi2c, DevAddress << 1, Trials, Timeout);
}

HAL_StatusTypeDef I2C_BUS::write(uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Transmit(_hi2c, DevAddress << 1, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_BUS::read(uint16_t DevAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Master_Receive(_hi2c, DevAddress << 1, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_BUS::memWrite8(uint16_t DevAddress, uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Write(_hi2c, DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_BUS::memWrite16(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Write(_hi2c, DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_16BIT, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_BUS::memRead8(uint16_t DevAddress, uint8_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Read(_hi2c, DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, Timeout);
}

HAL_StatusTypeDef I2C_BUS::memRead16(uint16_t DevAddress, uint16_t MemAddress, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    return HAL_I2C_Mem_Read(_hi2c, DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_16BIT, pData, Size, Timeout);
}

} // namespace ukaikokko
