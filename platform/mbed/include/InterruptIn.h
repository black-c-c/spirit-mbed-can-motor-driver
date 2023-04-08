#ifndef SPIRIT_MBED_INTERRUPTIN_H
#define SPIRIT_MBED_INTERRUPTIN_H

#include "include/InterfaceInterruptIn.h"
#include "mbed.h"

namespace spirit {

namespace mbed {

class InterruptIn : public InterfaceInterruptIn {
public:
    /**
     * @brief Constructor
     * @param pin InterruptIn pin to connect to
     */
    InterruptIn(PinName pin);

    /**
     * @brief Destructor
     */
    ~InterruptIn();

    /**
     * @brief 入力の立ち上がり時に呼び出す関数の設定
     *
     */
    void rise(std::function<void(void)> rise_func) override;

    /**
     * @brief 入力の立ち下がり時に呼び出す関数の設定
     *
     */
    void fall(std::function<void(void)> fall_func) override;

    /**
     * @brief 出力ピンの出力を返す
     * @retval 0 Low
     * @retval 1 High
     */
    uint32_t read() override;

private:
    ::mbed::InterruptIn _interrupt_in;

    std::function<void(void)> _rise_func;
    std::function<void(void)> _fall_func;
};

}  // namespace mbed

}  // namespace spirit

#endif  // SPIRIT_MBED_INTERRUPTIN_H
