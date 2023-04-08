#include "platform/mbed/include/InterruptIn.h"

#include "mbed.h"

namespace spirit {

namespace mbed {

InterruptIn::InterruptIn(PinName pin) : _interrupt_in(pin)
{
}

InterruptIn::~InterruptIn()
{
}

void InterruptIn::rise(std::function<void(void)> rise_func)
{
    _rise_func = rise_func;
    /// @todo mbedとtestで条件分け
    //_interrupt_in.rise(Callback<void()>(rise_func));
}

void InterruptIn::fall(std::function<void(void)> fall_func)
{
    _fall_func = fall_func;
    /// @todo mbedとtestで条件分け
    //_interrupt_in.fall(callback(&_fall_func));
}

uint32_t InterruptIn::read()
{
    return _interrupt_in.read();
}

}  // namespace mbed

}  // namespace spirit
