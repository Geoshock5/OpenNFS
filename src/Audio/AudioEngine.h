#pragma once

#include <iostream>

#include "al.h"
#include "alc.h"

class AudioEngine
{
public:
    AudioEngine();
    bool Init();
    bool Shutdown();

    ALuint uiBuffers;
    ALuint uiSource;
    ALuint uiBuffer;
    ALint iState;
    ALint iLoop;
    ALint iBuffersProcessed, iTotalBuffersProcessed, iQueuedBuffers;

private:
    ALCdevice *m_soundDevice   = 0;
    ALCcontext *m_soundContext = 0;
    ~AudioEngine();
};