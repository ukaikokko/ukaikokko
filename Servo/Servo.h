#ifndef __UKAIKOKKO_SERVO_H
#define __UKAIKOKKO_SERVO_H

#include <main.h>
#include "../PWM/PWM.h"

namespace ukaikokko
{

struct ServoParam
{
    double pos1_degree = 0.0;
    double pos1_pulseWidth_ms = 0.544;
    double pos2_degree = 180.0;
    double pos2_pulseWidth_ms = 2.400;
};

class Servo
{
   public:
    Servo(PWM pwm, ServoParam param = {}) : _pwm(pwm), _param(param) {}
    ~Servo() = default;
    /// @brief PWM出力を開始する
    HAL_StatusTypeDef start() const;
    /// @brief PWM出力を停止する
    HAL_StatusTypeDef stop() const;
    void setDegree(const double degree);
    void setPulseWidth_ms(const double pulseWidth_ms);

   private:
    PWM _pwm;
    const ServoParam _param;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_SERVO_H
