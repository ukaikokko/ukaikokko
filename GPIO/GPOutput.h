#ifndef __GPOUTPUT_H
#define __GPOUTPUT_H

#include <main.h>

namespace ukaikokko
{

class GPOutput
{
   public:
    GPOutput(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) : _GPIOx(GPIOx), _GPIO_Pin(GPIO_Pin) {}
    ~GPOutput() = default;
    void write(bool PinState) const;
    void write(GPIO_PinState PinState) const;
    void toggle() const;

   private:
    GPIO_TypeDef* _GPIOx;
    uint16_t _GPIO_Pin;
};

} // namespace ukaikokko

#endif // __GPOUTPUT_H
