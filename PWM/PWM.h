#ifndef __UKAIKOKKO_PWM_H
#define __UKAIKOKKO_PWM_H

#include <main.h>

namespace ukaikokko
{

class PWM
{
   public:
    PWM(TIM_HandleTypeDef* htim, uint32_t Channel) : _htim(htim), _Channel(Channel) {}
    ~PWM();
    HAL_StatusTypeDef start() const;
    HAL_StatusTypeDef stop() const;
    void setDuty(const double duty); // duty: 0.0 ~ 1.0

   private:
    TIM_HandleTypeDef* _htim;
    const uint32_t _Channel;
    double _duty = 0.0;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_PWM_H
