// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// #include "Task.h"
//
// #include "MultiMedia/BeepMusic.hpp"
//
// BeepMusic<BUZZER_PWM_ID> MusicBuzzer(0);
//
// enum class button_state_e {
//     UNPRESSED = 0 ,
//     PRESSED
// };
//
// constexpr unsigned int debounceChecks = 5;
// constexpr auto buttonPressedLevel = GPIO_PIN_RESET;
//
// static button_state_e buttonStableState = button_state_e::UNPRESSED;
// static button_state_e buttonLastState = button_state_e::UNPRESSED;
//
// static int songIndex = 0;
//
// void updateButtonStateBitwise(button_state_e* current_state, bool rawIsPressed) {
//     constexpr uint32_t debounceMask = (1U << debounceChecks) - 1;
//     static uint32_t history = 0;
//     history = (history << 1) | rawIsPressed;
//     if ((history & debounceMask) == debounceMask) {
//         *current_state = button_state_e::PRESSED;
//     } else if ((history & debounceMask) == 0) {
//         *current_state = button_state_e::UNPRESSED;
//     }
// }
//
// void TaskBeepMusic() {
//     bool rawIsPressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == buttonPressedLevel);
//     updateButtonStateBitwise(&buttonStableState,rawIsPressed );
//
//     if (buttonStableState != buttonLastState) {
//         if (buttonStableState == button_state_e::PRESSED) {
//             songIndex = (songIndex + 1) % 5;
//             MusicBuzzer.Play(songIndex);
//         }
//     }
//     buttonLastState = buttonStableState;
// }
// TASK_EXPORT(TaskBeepMusic);


#include "MultiMedia/BeepMusic.hpp"
#include "MultiMedia/MultiButton.hpp"

BeepMusic<BUZZER_PWM_ID> MusicBuzzer(0);

auto isPinPressed = [] -> bool {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET ) {
        return true;
    }
    return false;
};

void executeFunc(ButtonState state) {
    static int songIndex = 0;
    switch (state) {
        case ButtonState::StateIdle:
            break;
        case ButtonState::StatePress:
            break;
        case ButtonState::StateRelease:
            songIndex = (songIndex + 1) % 5;
            MusicBuzzer.Play(songIndex);
            break;
        case ButtonState::StateRepeat:
            MusicBuzzer.Play(3);
            break;
        case ButtonState::StateLongHold:
            MusicBuzzer.Play(4);
            break;
    }
}

Button userKey(isPinPressed,executeFunc);













