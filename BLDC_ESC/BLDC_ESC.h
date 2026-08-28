#ifndef __UKAIKOKKO_BLDC_ESC_H
#define __UKAIKOKKO_BLDC_ESC_H

#include <main.h>
#include "../PWM/PWM.h"

namespace ukaikokko
{

class BLDC_ESC
{
   public:
    BLDC_ESC(PWM pwm) : _pwm(pwm) {}
    ~BLDC_ESC() = default;
    /// @brief PWM出力を開始する
    HAL_StatusTypeDef start() const;
    /// @brief PWM出力を停止する
    HAL_StatusTypeDef stop() const;
    /// @param output 範囲 : 0.0 ~ 1.0
    double setOutput(const double output);

   private:
    constexpr static double MAX_OUTPUT = 0.1;  // 20ms*10%=2ms
    constexpr static double MIN_OUTPUT = 0.05; // 20ms*5%=1ms
    constexpr static double RANGE = MAX_OUTPUT - MIN_OUTPUT;
    void forceOutput(const double rawOutput);
    PWM _pwm;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_BLDC_ESC_H
