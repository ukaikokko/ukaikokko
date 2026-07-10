#include "GPInput.h"

GPIO_PinState GPInput::read() const
{
    return HAL_GPIO_ReadPin(_GPIOx, _GPIO_Pin);
}