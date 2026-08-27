#include "GPOutput.h"

namespace ukaikokko
{

void GPOutput::write(bool PinState) const
{
    if (PinState)
    {
        write(GPIO_PIN_SET);
    }
    else
    {
        write(GPIO_PIN_RESET);
    }
}

void GPOutput::write(GPIO_PinState PinState) const
{
    HAL_GPIO_WritePin(_GPIOx, _GPIO_Pin, PinState);
}

void GPOutput::toggle() const
{
    HAL_GPIO_TogglePin(_GPIOx, _GPIO_Pin);
}

} // namespace ukaikokko
