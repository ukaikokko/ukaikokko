#ifndef __UKAIKOKKO_LAMBCHAN_MD_H
#define __UKAIKOKKO_LAMBCHAN_MD_H

#include <main.h>
#include "../GPIO/GPOutput.h"
#include "../PWM/PWM.h"

namespace ukaikokko
{

struct MDParam
{
    int direction = 1;       // 1:正転, -1:逆転
    double maxOutput = 1.0;  // 最大出力
    double minOutput = -1.0; // 最小出力
    double maxAccel = 0.1;   // 最大加速度(output_diff/呼び出し周期)
};

class LambchanMD
{
   public:
    LambchanMD(PWM pwm, GPOutput dir, MDParam param)
        : _pwm(pwm), _dir(dir), _param(param) {}
    ~LambchanMD();
    HAL_StatusTypeDef start() const;
    HAL_StatusTypeDef stop() const;
    /// @param output 範囲 : -1.0 ~ 1.0
    double setOutput(const double output);

   private:
    void forceOutput(const double rawOutput);
    PWM _pwm;
    GPOutput _dir;
    const MDParam _param;
    double _output = 0.0;
};

} // namespace ukaikokko

#endif // __UKAIKOKKO_LAMBCHAN_MD_H
