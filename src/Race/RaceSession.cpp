#include "RaceSession.h"

#include <imgui.h>

RaceSession::RaceSession(const std::shared_ptr<GLFWwindow> &window,
                         const std::shared_ptr<Logger> &onfsLogger,
                         const std::vector<NfsAssetList> &installedNFS,
                         const std::shared_ptr<Track> &currentTrack,
                         const std::shared_ptr<Car> &currentCar) :
    m_window(window),
    m_track(currentTrack),
    m_playerAgent(std::make_shared<PlayerAgent>(window, currentCar, currentTrack)),
    m_renderer(window, onfsLogger, installedNFS, m_track, m_physicsEngine.debugDrawer)
{
    m_loadedAssets = {m_playerAgent->vehicle->tag, m_playerAgent->vehicle->id, m_track->nfsVersion, m_track->name};
    m_renderer.m_joyState = &m_joyState;
    m_playerAgent->m_joyState = &m_joyState;

    // Register an audio device and create a context
    m_soundDevice = alcOpenDevice(NULL);
    m_soundContext = alcCreateContext(m_soundDevice, NULL);
    if (m_soundDevice)
    {
        LOG(INFO) << "ALOutput : Using OpenAL Device " << alcGetString(m_soundDevice, ALC_DEVICE_SPECIFIER) << "\n";
        LOG(INFO) << "AL Context: " << m_soundContext << "\n";
        alcMakeContextCurrent(m_soundContext);

        //MusicLoader::MusicLoader("../resources/NFS_3/gamedata/audio/pc/alpirock");
        BnkLoader bnkFile;
        std::vector<AudioBuffer> horn = bnkFile.LoadBnk("assets/car/NFS_3/corv/car.bnk");
        LOG(INFO) << "Buffer has " << horn[0].GetHeaderPtr()->dwNumSamples << " samples of data. Buffer size " << sizeof(*horn[0].GetBufPtr()) << " bytes";

        /*
        // Open raw PCM stream
        musicstream = fopen("file.pcm", "rb");
        fseek(musicstream, 0, SEEK_END);
        long streamlength = ftell(musicstream);
        fseek(musicstream, 0, SEEK_SET);
        

        ALchar *musicData = new char[streamlength];
        fread(musicData, streamlength, 1, musicstream);
        */

        // Generate buffers and sources
        alGenBuffers(1, &uiBuffer);
        alGenSources(1, &uiSource);

        // Add file to buffer
        alBufferData(uiBuffer, AL_FORMAT_MONO16, horn[0].GetBufPtr(), horn[0].GetHeaderPtr()->dwNumSamples * horn[0].GetHeaderPtr()->dwBytesPerSample, horn[0].GetHeaderPtr()->dwSampleRate);
        
        // Attach source to buffer, queue and play
        alSourcei(uiSource, AL_BUFFER, uiBuffer);
        alSourcei(uiSource, AL_LOOPING, AL_TRUE);
        //alSourceQueueBuffers(uiSource, 1, &uiBuffer);
        alSourcePlay(uiSource);
    }
    
    // Set up the cameras
    m_freeCamera    = std::make_shared<FreeCamera>(m_window, m_track->trackBlocks[0].position);
    m_hermiteCamera = std::make_shared<HermiteCamera>(m_track->centerSpline, m_window);
    m_carCamera     = std::make_shared<CarCamera>(m_window);

    // Generate the collision meshes
    m_physicsEngine.RegisterTrack(m_track);

    // Set up the Racer Manager to spawn vehicles on track
    m_racerManager = RacerManager(m_playerAgent, m_track, m_physicsEngine);
}

void RaceSession::_UpdateCameras(float deltaTime)
{
    if (m_windowStatus == WindowStatus::GAME)
    {
        switch (m_activeCameraMode)
        {
        case FOLLOW_CAR:
            // Compute MVP from keyboard and mouse, centered around a target car
            m_carCamera->FollowCar(m_playerAgent->vehicle);
            break;
        case HERMITE_FLYTHROUGH:
            m_hermiteCamera->UseSpline(m_totalTime);
            break;
        case FREE_LOOK:
            // Compute the MVP matrix from keyboard and mouse input
            m_freeCamera->ComputeMatricesFromInputs(deltaTime);
            break;
        }
    }
}

std::shared_ptr<BaseCamera> RaceSession::_GetActiveCamera()
{
    if (m_userParams.attachCamToHermite)
    {
        m_activeCameraMode = CameraMode::HERMITE_FLYTHROUGH;
        return m_hermiteCamera;
    }
    else if (m_userParams.attachCamToCar)
    {
        m_activeCameraMode = CameraMode::FOLLOW_CAR;
        return m_carCamera;
    }
    else
    {
        m_activeCameraMode = CameraMode::FREE_LOOK;
        return m_freeCamera;
    }
}

AssetData RaceSession::Simulate()
{
    while (!glfwWindowShouldClose(m_window.get()))
    {
        // glfwGetTime is called only once, the first time this function is called
        static double lastTime = glfwGetTime();
        // Compute time difference between current and last frame
        double currentTime = glfwGetTime();
        // Update time between engine ticks
        auto deltaTime = float(currentTime - lastTime); // Keep track of time between engine ticks

        // Clear the screen for next input and grab focus
        this->_GetInputsAndClear();

        // Update Cameras
        this->_UpdateCameras(deltaTime);

        // Set the active camera dependent upon user input
        std::shared_ptr<BaseCamera> activeCamera = this->_GetActiveCamera();

        if (m_userParams.simulateCars)
        {
            m_racerManager.Simulate();
        }

        m_orbitalManager.Update(activeCamera, m_userParams.timeScaleFactor);

        // Step the physics simulation
        m_physicsEngine.StepSimulation(deltaTime, m_racerManager.GetRacerResidentTrackblocks());
        if (m_userParams.physicsDebugView)
        {
            m_physicsEngine.GetDynamicsWorld()->debugDrawWorld();
        }

        bool assetChange =
          m_renderer.Render(m_totalTime, activeCamera, m_hermiteCamera, m_orbitalManager.GetActiveGlobalLight(), m_userParams, m_loadedAssets, m_racerManager.racers);

        /*
         * TODO: Need to move this inside of the main render, else IMGUI will bug out as frame has ended
         * if (ImGui::GetIO().MouseReleased[0] && m_windowStatus == WindowStatus::GAME)
        {
            bool entityTargeted = false;
            Entity *targetedEntity = m_physicsEngine.CheckForPicking(activeCamera->viewMatrix, activeCamera->projectionMatrix,
        entityTargeted); if (entityTargeted)
            {
                Renderer::DrawMetadata(targetedEntity);
            }
        }*/

        if (assetChange)
        {
            return m_loadedAssets;
        }

        // For the next frame, the "last time" will be "now"
        lastTime = currentTime;
        // Keep track of total elapsed time too
        m_totalTime += deltaTime;
        ++m_ticks;
    }

    // Close sound device
    if (m_soundDevice)
    {
        alcMakeContextCurrent(NULL);
        alcDestroyContext(m_soundContext);
        alcCloseDevice(m_soundDevice);
        m_soundDevice = 0;

        fclose(musicstream);
    }

    // Just set a flag temporarily to let main know that we outta here
    m_loadedAssets.trackTag = UNKNOWN;
    return m_loadedAssets;
}

void RaceSession::_GetInputsAndClear()
{
    glClearColor(0.1f, 0.f, 0.5f, 1.f);
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Update joystick state
    m_joyState.joy0axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &m_joyState.joy0axcount);
    m_joyState.joy0buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &m_joyState.joy0butcount);

    // Detect a click on the 3D Window by detecting a click that isn't on ImGui
    if ((glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) && (!ImGui::GetIO().WantCaptureMouse))
    {
        m_windowStatus                 = WindowStatus::GAME;
        ImGui::GetIO().MouseDrawCursor = false;
    }
    else if (glfwGetKey(m_window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        m_windowStatus                 = WindowStatus::UI;
        ImGui::GetIO().MouseDrawCursor = true;
    }
}
