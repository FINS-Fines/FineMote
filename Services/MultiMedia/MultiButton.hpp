
#ifndef MULTI_BUTTON_HPP
#define MULTI_BUTTON_HPP// ButtonDebounce.hpp
#include <cstdint>
#include <functional>
#include <utility>
#include "MultiMedia/BeepMusic.hpp"
#include "MultiMedia/MultiButton.hpp"

static int songIndex = 0;
BeepMusic<BUZZER_PWM_ID> MusicBuzzer(0);

constexpr uint32_t SHORT_TICKS = 200 ;
constexpr uint32_t LONG_TICKS = 1000 ;
constexpr uint32_t GAP_TICKS = 400 ;
constexpr uint8_t PRESS_REPEAT_MAX_NUM = 2;

class Button;
using ButtonCallback = void (*)(Button*);
// enum class ButtonEvent : uint8_t {
//     BTN_PRESS_DOWN = 0,
//     BTN_PRESS_UP,
//     BTN_PRESS_REPEAT,
//     BTN_SINGLE_CLICK,
//     BTN_DOUBLE_CLICK,
//     BTN_LONG_PRESS_START,
//     BTN_LONG_PRESS_HOLD,
//     BTN_EVENT_COUNT,
//     BTN_NONE_PRESS
// };
enum class ButtonState : uint8_t {
    BTN_STATE_IDLE = 0,
    BTN_STATE_PRESS,
    BTN_STATE_RELEASE,
    BTN_STATE_REPEAT,
    BTN_STATE_LONG_HOLD
};
class Button : public DeviceBase{
public:
    template <typename conditionFunc>
    Button (conditionFunc&& condition ) : isButtonPressed(std::forward<conditionFunc>(condition)) {
        // SetDivisionFactor(5);
        ticks = 0;
        gapTicks = 0;
        // repeat = 0;
        isButtonBeingPressed = false;
        state = ButtonState::BTN_STATE_IDLE;
        // event = ButtonEvent::BTN_NONE_PRESS;
    }
    void Handle() final {
        ticks++;
        if (isButtonPressed()) isButtonBeingPressed = true;
        else isButtonBeingPressed = false;
        switch (state) {
            case ButtonState::BTN_STATE_IDLE:
                Implement_Idle();
                break;
            case ButtonState::BTN_STATE_PRESS:
                Implement_Press();
                break;
            case ButtonState::BTN_STATE_RELEASE:
                Implement_Release();
                break;
            case ButtonState::BTN_STATE_REPEAT:
                Implement_Repeat();
                break;
            case ButtonState::BTN_STATE_LONG_HOLD:
                Implement_LongHold();
                break;
        }
    }

    void Implement_Idle() {
        if (isButtonBeingPressed) {
            if (ticks > SHORT_TICKS) {
                state = ButtonState::BTN_STATE_PRESS;
            }
        }
        else {
            ticks = 0;
        }
    }
    void Implement_Press() {
        if (!isButtonBeingPressed) {
            state = ButtonState::BTN_STATE_RELEASE;
            ticks = 0;
        }
        else {
            if (ticks > LONG_TICKS) {
                state = ButtonState::BTN_STATE_LONG_HOLD;
            }
        }
    }
    void Implement_Release() {
        gapTicks++;
        if (gapTicks > GAP_TICKS) {
            ticks = 0;
            gapTicks = 0;
            songIndex = (songIndex + 1) % 5;
            MusicBuzzer.Play(songIndex);                        // 单击 判定完毕 播放 songIndex
        }
        else {
            if (isButtonBeingPressed) {
                if (ticks > SHORT_TICKS) {
                    gapTicks = 0;
                    state = ButtonState::BTN_STATE_REPEAT;
                }
            }
            else {
                ticks = 0;
            }
        }
    }
    void Implement_Repeat() {
        if (!isButtonBeingPressed) {
            MusicBuzzer.Play(3);                       // 双击 判定完毕 播放index 3
            ticks = 0;
            state = ButtonState::BTN_STATE_IDLE;
        }
    }

    void Implement_LongHold() {
        if (!isButtonBeingPressed) {
            ticks = 0;
            state = ButtonState::BTN_STATE_RELEASE;
            MusicBuzzer.Play(4);                         // 长按 判定完毕 播放 index 4
        }
    }

    std::function<bool()> isButtonPressed;

    int ticks ;
    int gapTicks;
    // int repeat ;
    bool isButtonBeingPressed;
    // ButtonEvent event ;
    ButtonState state ;
};
#endif


