#ifndef __UKAIKOKKO_ANALOG_INPUT_DMA_H
#define __UKAIKOKKO_ANALOG_INPUT_DMA_H

#include <main.h>

namespace ukaikokko
{

class AnalogInputDMA
{
   public:
    AnalogInputDMA(ADC_HandleTypeDef* hadc, uint32_t* dma_buf, uint32_t length) : _hadc(hadc), _dma_buf(dma_buf), _length(length) {}
    HAL_StatusTypeDef start() const;
    /// @brief
    /// @param index 0 ~ length - 1
    /// @return Voltage value (0.0 ~ 3.3)
    double read(uint32_t index) const;

   private:
    ADC_HandleTypeDef* _hadc;
    uint32_t* _dma_buf;
    uint32_t _length;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_ANALOG_INPUT_DMA_H
