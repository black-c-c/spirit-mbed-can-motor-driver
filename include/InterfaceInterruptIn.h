#ifndef SPIRIT_INTERFACE_INTERRUPTIN_H
#define SPIRIT_INTERFACE_INTERRUPTIN_H

#include <functional>
#include <cstdint>

namespace spirit {

/**
 * @brief mbedのInterruptInクラスを模した基底クラス
 */
class InterfaceInterruptIn {
public:
    /**
     * @brief デストラクタ
     */
    virtual ~InterfaceInterruptIn() = default;

    /**
     * @brief 入力の立ち上がり時に呼び出す関数の設定
     *
     */
    virtual void rise(std::function<void(void)> func) = 0;

    /**
     * @brief 入力の立ち下がり時に呼び出す関数の設定
     *
     */
    virtual void fall(std::function<void(void)> func) = 0;

    /**
     * @brief 入力ピンの入力を返す
     * @retval 0 Low
     * @retval 1 High
     */
    virtual uint32_t read() = 0;
};

}  // namespace spirit

#endif  //SPIRIT_INTERFACE_INTERRUPTIN_H
