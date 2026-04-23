#pragma once

#include <iostream>

#include "AL/al.h"
#include "AL/alc.h"

class AudioEngine
{
public:
    AudioEngine();

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