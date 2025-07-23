
#ifndef MULTI_BUTTON_HPP
#define MULTI_BUTTON_HPP
#include <cstdint>
#include <functional>
#include <utility>
#include "MultiMedia/BeepMusic.hpp"
#include "MultiMedia/MultiButton.hpp"

constexpr uint32_t SHORT_TICKS = 200 ;
constexpr uint32_t LONG_TICKS = 1000 ;
constexpr uint32_t GAP_TICKS = 400 ;
constexpr uint8_t PRESS_REPEAT_MAX_NUM = 2;

class Button;
using ButtonCallback = void (*)(Button*);
// enum class ButtonEvent : uint8_t {
//     PressDown = 0,
//     PressUp,
//     PressRepeat,
//     SingleClick,
//     DoubleClick,
//     LongPressStart,
//     LongPressHold,
//     NonePress
// };
enum class ButtonState : uint8_t {
    StateIdle = 0,
    StatePress,
    StateRelease,
    StateRepeat,
    StateLongHold
};
class Button : public DeviceBase{
public:
    template <typename conditionFunc , typename exeFunc>
    Button (conditionFunc&& condition , exeFunc&& execute) : IsButtonPressed(std::forward<conditionFunc>(condition)) ,ExecuteFunction(std::forward<exeFunc>(execute)){
        SetDivisionFactor(5);
        ticks = 0;
        gapTicks = 0;
        // repeat = 0;
        isButtonBeingPressed = false;
        state = ButtonState::StateIdle;
        // event = ButtonEvent::BTN_NONE_PRESS;
    }
    void Handle() final {
        ticks++;
        if (IsButtonPressed()) isButtonBeingPressed = true;
        else isButtonBeingPressed = false;
        switch (state) {
            case ButtonState::StateIdle:
                ImplementIdle();
                break;
            case ButtonState::StatePress:
                ImplementPress();
                break;
            case ButtonState::StateRelease:
                ImplementRelease();
                break;
            case ButtonState::StateRepeat:
                ImplementRepeat();
                break;
            case ButtonState::StateLongHold:
                ImplementLongHold();
                break;
        }
    }

    void ImplementIdle() {
        if (isButtonBeingPressed) {
            if (ticks > SHORT_TICKS) {
                state = ButtonState::StatePress;
            }
        }
        else {
            ticks = 0;
        }
    }
    void ImplementPress() {
        if (!isButtonBeingPressed) {
            state = ButtonState::StateRelease;
            ticks = 0;
        }
        else {
            if (ticks > LONG_TICKS) {
                state = ButtonState::StateLongHold;
            }
        }
    }
    void ImplementRelease() {
        gapTicks++;
        if (gapTicks > GAP_TICKS) {
            ticks = 0;
            gapTicks = 0;
            ExecuteFunction(ButtonState::StateRelease);  // 单击 判定完毕 播放 songIndex
        }
        else {
            if (isButtonBeingPressed) {
                if (ticks > SHORT_TICKS) {
                    gapTicks = 0;
                    state = ButtonState::StateRepeat;
                }
            }
            else {
                ticks = 0;
            }
        }
    }
    void ImplementRepeat() {
        if (!isButtonBeingPressed) {
            ExecuteFunction(ButtonState::StateRepeat);    // 双击 判定完毕 播放index 3
            ticks = 0;
            state = ButtonState::StateIdle;
        }
    }
    void ImplementLongHold() {
        if (!isButtonBeingPressed) {
            ticks = 0;
            state = ButtonState::StateRelease;
            ExecuteFunction(ButtonState::StateLongHold);  // 长按 判定完毕 播放 index 4
        }
    }

    std::function<bool()> IsButtonPressed;
    std::function<void(ButtonState)> ExecuteFunction;

    int ticks ;
    int gapTicks;
    // int repeat ;
    bool isButtonBeingPressed;
    // ButtonEvent event ;
    ButtonState state ;
};
#endif


