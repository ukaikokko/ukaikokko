#ifndef __GPINPUT_H
#define __GPINPUT_H

#include <main.h>

class GPInput
{
   public:
    GPInput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) : _GPIOx(GPIOx), _GPIO_Pin(GPIO_Pin) {}
    ~GPInput() = default;
    GPIO_PinState read() const;

   private:
    GPIO_TypeDef* _GPIOx;
    uint16_t _GPIO_Pin;
};

#endif // __GPINPUT_H
