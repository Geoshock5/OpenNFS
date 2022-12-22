#pragma once

#include "CarAgent.h"
#include "../../Config.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

class PlayerAgent : public CarAgent
{
public:
    PlayerAgent(const std::shared_ptr<GLFWwindow> &window, const std::shared_ptr<Car> &car, const std::shared_ptr<Track> &raceTrack);
    void Simulate() override;

    JoyState *m_joyState;
    JoyState lastJoyState;

private:
    std::shared_ptr<GLFWwindow> m_window;

    //int *count;
    //const unsigned char *hats;
};
