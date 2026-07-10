#include "GPInput.h"

namespace ukaikokko
{

GPIO_PinState GPInput::read() const
{
    return HAL_GPIO_ReadPin(_GPIOx, _GPIO_Pin);
}

} // namespace ukaikokko