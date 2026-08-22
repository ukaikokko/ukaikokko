#include "PWM.h"

namespace ukaikokko
{

PWM::~PWM()
{
    stop();
}

HAL_StatusTypeDef PWM::start() const
{
    return HAL_TIM_PWM_Start(_htim, _Channel);
}

HAL_StatusTypeDef PWM::stop() const
{
    return HAL_TIM_PWM_Stop(_htim, _Channel);
}

void PWM::setDuty(const double duty)
{
    if (duty < 0.0)
    {
        _duty = 0.0;
    }
    else if (duty > 1.0)
    {
        _duty = 1.0;
    }
    else
    {
        _duty = duty;
    }

    if (_htim->Init.Period == 65535 || _htim->Init.Period == 4294967295)
    {
        // 1を足すとオーバーフローしてしまう // 他にいいやり方があったら教えてほしいです！
        const uint32_t ccr = static_cast<uint32_t>(static_cast<double>(_htim->Init.Period) * _duty);
        __HAL_TIM_SetCompare(_htim, _Channel, ccr);
    }
    else
    {
        const uint32_t ccr = static_cast<uint32_t>(static_cast<double>(_htim->Init.Period + 1) * _duty);
        __HAL_TIM_SetCompare(_htim, _Channel, ccr);
    }
}

} // namespace ukaikokko
