#include "PIDSpeedController.h"

namespace spirit {
    
PIDSpeedController::PIDSpeedController(InterfaceInterruptIn& in_a, InterfaceInterruptIn& in_b, uint8_t ppr)
    : _in_a(in_a), _in_b(in_b)
{
    digree_unit = 360.0f/(4.0f*ppr);
    reset();

    _phase_a = std::bind(&PIDSpeedController::phase_a, this);
    _phase_b = std::bind(&PIDSpeedController::phase_b, this);

    _in_a.rise(_phase_a);
    _in_b.rise(_phase_b);
    _in_a.fall(_phase_a);
    _in_b.fall(_phase_b);
}

void PIDSpeedController::set_parameter(float tem_kp, float tem_ki, float tem_kd)
{
    _kp = tem_kp;
    _ki = tem_ki;
    _kd = tem_kd;
}

void PIDSpeedController::set_target(float target)
{
    _target = target;
}

void PIDSpeedController::set_limit(float low_limit, float high_limit)
{
    _low_limit  = low_limit;
    _high_limit = high_limit;
}

float PIDSpeedController::process(float _dt)
{
    deviation = _target - rps(_dt);

    sum_deviation  += deviation;
    delta_deviation = deviation - Last_deviation;

    float _p = _kp * deviation;
    float _i = _ki * sum_deviation;
    float _d = _kd * delta_deviation;

    Last_deviation = deviation;

    limiter(&_i, _low_limit, _high_limit);

    mv = _p + _i + _d;

    limiter(&mv, _low_limit, _high_limit);

    return mv;
}

void PIDSpeedController::limiter(float* val, float low_limit, float high_limit)
{
    if(*val > high_limit) {
        *val = high_limit;
    } else if(*val < low_limit) {
        *val = low_limit;
    }
}

void PIDSpeedController::start()
{
    lock_start = true;
}

void PIDSpeedController::stop()
{
    lock_start = false;
}

void PIDSpeedController::reset()
{
    loop_counter    = 0;
    buff[0]         = 0;
    angle           = 0.0f;
    deviation       = 0.0f;
    Last_deviation  = 0.0f;
    sum_deviation   = 0.0f;
    delta_deviation = 0.0f;
}

void PIDSpeedController::phase_a()
{
    if(lock_start) {
        if(_in_a.read() == 1) {
            if(_in_b.read() == 1) {
                angle += digree_unit;
            } else {
                angle -= digree_unit;
            }
        } else {
            if(_in_b.read() == 0) {
                angle += digree_unit;
            } else {
                angle -= digree_unit;
            }
        }
    }
}

void PIDSpeedController::phase_b()
{
    if(lock_start) {
        if(_in_b.read() == 1) {
            if(_in_a.read() == 0) {
                angle += digree_unit;
            } else {
                angle -= digree_unit;
            }
        } else {
            if(_in_a.read() == 1) {
                angle += digree_unit;
            } else {
                angle -= digree_unit;
            }
        }
    }
}

float PIDSpeedController::rps(float _dt)
{
    if(loop_counter == MAX_BUFF) {
        for(int i=0; i<(MAX_BUFF-1); i++) {
            buff[i] = buff[i+1];
        }
        buff[MAX_BUFF-1] = angle;
        return (1.0f/_dt * (buff[MAX_BUFF-1] - buff[0]) / (float)(MAX_BUFF - 1)) / 360.0f;
    } else {
        loop_counter++;
        buff[loop_counter] = angle;
        return (1.0f/_dt * (buff[loop_counter] - buff[0]) / (float)loop_counter) / 360.0f;
    }
}

}  // namespace spirit
