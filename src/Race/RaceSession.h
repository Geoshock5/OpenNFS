#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "../Camera/FreeCamera.h"
#include "../Camera/HermiteCamera.h"
#include "../Camera/CarCamera.h"
#include "../Physics/PhysicsEngine.h"
#include "../Loaders/CarLoader.h"
#include "../Scene/Track.h"
#include "../Renderer/Renderer.h"
#include "../RaceNet/Agents/PlayerAgent.h"
#include "../Util/Logger.h"
#include "../Config.h"
#include "RacerManager.h"
#include "OrbitalManager.h"
#include "al.h"
#include "alc.h"
#include "../Audio/AudioEngine.h"
#include "../Loaders/MusicLoader.h"

class RaceSession
{
public:
    RaceSession(const std::shared_ptr<GLFWwindow> &window,
                const std::shared_ptr<Logger> &onfsLogger,
                const std::vector<NfsAssetList> &installedNFS,
                const std::shared_ptr<Track> &currentTrack,
                const std::shared_ptr<Car> &currentCar);
    AssetData Simulate();

private:
    std::shared_ptr<BaseCamera> _GetActiveCamera();
    void _UpdateCameras(float deltaTime);
    void _GetInputsAndClear();

    AssetData m_loadedAssets;
    WindowStatus m_windowStatus   = WindowStatus::GAME;
    CameraMode m_activeCameraMode = CameraMode::FREE_LOOK;

    std::shared_ptr<GLFWwindow> m_window;
    std::shared_ptr<Track> m_track;
    std::shared_ptr<PlayerAgent> m_playerAgent;
    std::shared_ptr<FreeCamera> m_freeCamera;
    std::shared_ptr<HermiteCamera> m_hermiteCamera;
    std::shared_ptr<CarCamera> m_carCamera;

    PhysicsEngine m_physicsEngine;
    Renderer m_renderer;
    RacerManager m_racerManager;
    OrbitalManager m_orbitalManager;

    /* DIsable audio stuff for now
    ALuint uiBuffers;
    ALuint uiSource;
    ALuint uiBuffer;
    ALint iState;
    ALint iLoop;
    ALint iBuffersProcessed, iTotalBuffersProcessed, iQueuedBuffers;
    WAVEFORMATEX wfex;
    unsigned long ulDataSize  = 0;
    unsigned long ulFrequency = 0;
    unsigned long ulFormat    = 0;
    unsigned long ulBufferSize;
    unsigned long ulBytesWritten;
    void *pData = NULL;
    */

    ParamData m_userParams;
    JoyState m_joyState;
    uint64_t m_ticks  = 0; // Engine ticks elapsed
    float m_totalTime = 0;
};
