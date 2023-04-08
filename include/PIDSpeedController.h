#ifndef PID_SPEED_CONTROLLER_H
#define PID_SPEED_CONTROLLER_H

#include "InterfaceInterruptIn.h"
#include <cstdint>

namespace spirit {

class PIDSpeedController {
public:

    PIDSpeedController(InterfaceInterruptIn& in_a, InterfaceInterruptIn& in_b, uint8_t ppr);

    void set_parameter(float tem_kp, float tem_ki, float tem_kd);       // パラメータ変更関数
    void set_target(float target);                                      // 目標値設定関数
    void set_limit(float low_limit, float high_limit);                  // 操作量の上下限値設定関数
    float process(float _dt);                                         // PID処理関数
    void start();
    void stop();
    void reset();

private:
    InterfaceInterruptIn& _in_a;
    InterfaceInterruptIn& _in_b;

    std::function<void(void)> _phase_a;
    std::function<void(void)> _phase_b;

    static const uint8_t MAX_BUFF{10};

    bool lock_start{false};

    float _kp{0.0f};                // 比例パラメータ
    float _ki{0.0f};                // 積分パラメータ
    float _kd{0.0f};                // 微分パラメータ

    float _target{0.0f};            // 目標値

    float _low_limit{0.0f};         // 操作量の下限値
    float _high_limit{0.0f};        // 操作量の上限値

    float angle;
    float digree_unit;

    float buff[MAX_BUFF];

    int loop_counter;
    float mv;                       // 操作量
    float deviation;                // 偏差
    float Last_deviation;           // 前回の偏差
    float sum_deviation;            // 積分用合計偏差
    float delta_deviation;          // 微分用偏差の差

    void limiter(float* val, float low_limit, float high_limit);    // 操作量に上下のリミッターをかける関数
    void phase_a();
    void phase_b();
    float rps(float _dt);
};

}  // namespace spirit

#endif  // PID_SPEED_CONTROLLER_H
