/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
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