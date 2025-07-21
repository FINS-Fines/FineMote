/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#define FINEMOTE_BEEPMUSIC_C

#include "BeepMusic.h"
#include "PWM_Base.hpp"
#ifdef BEEPMUSIC_MODULE

/*****   曲库   *****/
#include "Music_Astronomia.h"
#include "Music_IronTorrent.h"
#include "Music_Senbonzakura.h"
#include "Music_SuperMario.h"
#include "Music_Windmill.h"
#include "Music_LiuSha.h"
#include "Music_YMCA.h"

Song_t BeepMusic::songs[] = {
    SONG_INIT(Astronomia),
    SONG_INIT(IronTorrent),
    SONG_INIT(Senbonzakura),
    SONG_INIT(SuperMario),
    SONG_INIT(Windmill),
    SONG_INIT(LiuSha),
    SONG_INIT(YMCA),
};

BeepMusic::BeepMusic(void (*handle)(uint32_t freq)) {
    PeripheralsInit::GetInstance();
    beepHandle = handle;
}

void BeepMusic::Play(int16_t songIndex) {
    curSong = songIndex;
    curNote = const_cast<MusicNote_t*>(songs[curSong].script);
    beepHandle(songs[curSong].script->Note);
}

void BeepMusic::Stop() {
    curSong = -1;
    beepHandle(0);
}

void BeepMusic::BeepService() {
    static uint32_t time = 0;
    if (curSong != -1) {
        if (time >= curNote->Time) {
            time = 0;
            curNote++;
            if(curNote - const_cast<MusicNote_t*>(songs[curSong].script) >= songs[curSong].noteNum) {
                Stop();
            } else {
                beepHandle(curNote->Note);
            }
        }

        time++;
    }
}

BeepMusic BeepMusic::MusicChannels[] = {
    BeepMusic([](uint32_t freq)->void {
        if(0 == freq){
            PWM_Base<BUZZER_PWM_ID>::GetInstance().SetDutyCycle(0);
        } else {
       //     PWM_Base<BUZZER_PWM_ID>::GetInstance().SetFrequency(freq);
        //    PWM_Base<BUZZER_PWM_ID>::GetInstance().SetDutyCycle(0.5);

        }
    }),
};
#endif