#include "AudioEngine.h"

#define NULL 0

AudioEngine::AudioEngine()
    {
        // Register an audio device and create a context
        m_soundDevice  = alcOpenDevice(NULL);
        m_soundContext = alcCreateContext(m_soundDevice, NULL);
        alcMakeContextCurrent(m_soundContext);

        if (alGetError()==0)
        {
            std::cout << "Device " << m_soundDevice << " opened successfully with context " << m_soundContext;
        }
        else
        {
            std::cout << "Error initialising sound device and context.\n";
        }
    };

AudioEngine::~AudioEngine()
{
    alcDestroyContext(m_soundContext);
    alcCloseDevice(m_soundDevice);
}