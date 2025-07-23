/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

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

class Button;
using ButtonCallback = void (*)(Button*);

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
        isButtonBeingPressed = false;
        state = ButtonState::StateIdle;
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
            state = ButtonState::StateIdle;
            ExecuteFunction(ButtonState::StateRelease);
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
            ExecuteFunction(ButtonState::StateRepeat);
            ticks = 0;
            state = ButtonState::StateIdle;
    }
    void ImplementLongHold() {
        if (!isButtonBeingPressed) {
            ticks = 0;
            state = ButtonState::StateRelease;
            ExecuteFunction(ButtonState::StateLongHold);
        }
    }

    std::function<bool()> IsButtonPressed;
    std::function<void(ButtonState)> ExecuteFunction;

    int ticks ;
    int gapTicks;
    bool isButtonBeingPressed;
    ButtonState state ;
};
#endif