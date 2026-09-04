#include "BLDC_ESC.h"

namespace ukaikokko
{

HAL_StatusTypeDef BLDC_ESC::start() const
{
    return _pwm.start();
}

HAL_StatusTypeDef BLDC_ESC::stop() const
{
    return _pwm.stop();
}

double BLDC_ESC::setOutput(const double output)
{
    double targetOutput = output;
    // 最大出力clamp
    if (targetOutput > 1.0)
    {
        targetOutput = 1.0;
    }
    else if (targetOutput < 0.0)
    {
        targetOutput = 0.0;
    }

    forceOutput(targetOutput);
    return targetOutput;
}

void BLDC_ESC::forceOutput(const double rawOutput)
{
    _pwm.setDuty(MIN_OUTPUT + rawOutput * RANGE);
}

} // namespace ukaikokko
