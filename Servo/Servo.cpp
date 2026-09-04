#include "Servo.h"

namespace ukaikokko
{

HAL_StatusTypeDef Servo::start() const
{
    return _pwm.start();
}

HAL_StatusTypeDef Servo::stop() const
{
    return _pwm.stop();
}

void Servo::setDegree(const double degree)
{
    if (_param.pos1_degree == _param.pos2_degree)
    {
        // 0除算防止
        setPulseWidth_ms(_param.pos1_pulseWidth_ms);
        return;
    }
    const double ratio = (degree - _param.pos1_degree) / (_param.pos2_degree - _param.pos1_degree);
    const double pulseWidth_ms = _param.pos1_pulseWidth_ms + ratio * (_param.pos2_pulseWidth_ms - _param.pos1_pulseWidth_ms);
    setPulseWidth_ms(pulseWidth_ms);
}

void Servo::setPulseWidth_ms(const double pulseWidth_ms)
{
    const double duty = pulseWidth_ms / 20.0; // 20ms周期
    _pwm.setDuty(duty);
}

} // namespace ukaikokko
