#include "PWM.h"

namespace ukaikokko
{

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

    // CCR計算(詳しくはREADME.mdを参照)
    if (_htim->Init.Period == UINT16_MAX || _htim->Init.Period == UINT32_MAX)
    {
        // 他にいいやり方があったら教えてほしいです！
        uint64_t ccr = static_cast<uint64_t>((static_cast<double>(_htim->Init.Period) + 1.0) * _duty);
        if (_htim->Init.Period == UINT16_MAX && ccr > UINT16_MAX)
        {
            ccr = UINT16_MAX;
        }
        else if (_htim->Init.Period == UINT32_MAX && ccr > UINT32_MAX)
        {
            ccr = UINT32_MAX;
        }
        __HAL_TIM_SetCompare(_htim, _Channel, ccr);
    }
    else
    {
        const uint32_t ccr = static_cast<uint32_t>(static_cast<double>(_htim->Init.Period + 1) * _duty);
        __HAL_TIM_SetCompare(_htim, _Channel, ccr);
    }
}

} // namespace ukaikokko
