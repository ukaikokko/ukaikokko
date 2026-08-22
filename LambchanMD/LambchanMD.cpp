#include "LambchanMD.h"

namespace ukaikokko
{

HAL_StatusTypeDef LambchanMD::start() const
{
    return _pwm.start();
}

HAL_StatusTypeDef LambchanMD::stop() const
{
    _dir.write(GPIO_PIN_RESET);
    return _pwm.stop();
}

double LambchanMD::setOutput(const double output)
{
    double targetOutput = output;
    // 最大出力clamp
    if (targetOutput > _param.maxOutput)
    {
        targetOutput = _param.maxOutput;
    }
    else if (targetOutput < _param.minOutput)
    {
        targetOutput = _param.minOutput;
    }
    // 最大加速度clamp
    const double nextMaxOutput = _output + _param.maxAccel;
    const double nextMinOutput = _output - _param.maxAccel;
    if (targetOutput > nextMaxOutput)
    {
        targetOutput = nextMaxOutput;
    }
    else if (targetOutput < nextMinOutput)
    {
        targetOutput = nextMinOutput;
    }

    _output = targetOutput;
    forceOutput(_output * _param.direction); // 方向を反映
    return _output;
}

void LambchanMD::forceOutput(const double rawOutput)
{
    if (rawOutput < 0.0)
    {
        _dir.write(GPIO_PIN_RESET);
        _pwm.setDuty(-rawOutput);
    }
    else
    {
        _dir.write(GPIO_PIN_SET);
        _pwm.setDuty(rawOutput);
    }
}

} // namespace ukaikokko
