#include "AnalogInputDMA.h"

namespace ukaikokko
{

HAL_StatusTypeDef AnalogInputDMA::start() const
{
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(_hadc, _dma_buf, _length);
    if (status == HAL_OK)
    {
        // 想定するDMAのバッファはちょうどlengthぶんしかないので，コールバックは不要
        __HAL_DMA_DISABLE_IT(_hadc->DMA_Handle, DMA_IT_HT); // half transfer 無効にしてCPU負荷を減らす
        __HAL_DMA_DISABLE_IT(_hadc->DMA_Handle, DMA_IT_TC); // transfer complete 無効にしてCPU負荷を減らす
    }
    return status;
}

double AnalogInputDMA::read(uint32_t index) const
{
    if (index < 0 || _length <= index)
    {
        return 0.0;
    }
    return (double)_dma_buf[index] * 3.3 / 4095.0; // 12bit ADC
}

} // namespace ukaikokko
