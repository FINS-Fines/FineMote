// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "Task.h"

#include "MultiMedia/BeepMusic.h"

void TaskBeepMusic() {
    if(BUTTON_PRESSED_STATE == HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)) {
        static int index = 1;
        BeepMusic::MusicChannels[0].Play(index++);
        index %= 3;
    }
    BeepMusic::MusicChannels[0].BeepService();
}
TASK_EXPORT(TaskBeepMusic);
