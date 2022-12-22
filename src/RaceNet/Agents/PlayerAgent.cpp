#include "PlayerAgent.h"

PlayerAgent::PlayerAgent(const std::shared_ptr<GLFWwindow> &window, const std::shared_ptr<Car> &car, const std::shared_ptr<Track> &raceTrack) :
    CarAgent(AgentType::PLAYER, car, raceTrack), m_window(window)
{
    name = "DumbPanda";
    //const unsigned char *hats = glfwGetJoystickHats(GLFW_JOYSTICK_1, count);
    //lastJoyState = JoyState();
}

void PlayerAgent::Simulate()
{
    // Update data required for efficient track physics update
    this->_UpdateNearestTrackblock();
    this->_UpdateNearestVroad();

    // Handle joystick input
    if (m_joyState->joy0axcount > 0)
    {
        vehicle->GearChange(m_joyState->joy0buttons[1],m_joyState->joy0buttons[2]);
        vehicle->ApplyAccelerationForce((float)((m_joyState->joy0axes[5] + 1.0f) / 2.0f), (float)((m_joyState->joy0axes[4] + 1.0f) / 2.0f));
        vehicle->ApplyBrakingForce(m_joyState->joy0buttons[0]);
        vehicle->ApplyAnalogSteering(m_joyState->joy0axes[0]);
        //lastJoyState = *m_joyState;
    }
    // if (userParams.windowActive && !ImGui::GetIO().MouseDown[1]) { }
    if (m_joyState->joy0axcount == NULL)
    {
        vehicle->ApplyAccelerationForce(glfwGetKey(m_window.get(), GLFW_KEY_W) == GLFW_PRESS, glfwGetKey(m_window.get(), GLFW_KEY_S) == GLFW_PRESS);
        vehicle->ApplyBrakingForce(glfwGetKey(m_window.get(), GLFW_KEY_SPACE) == GLFW_PRESS);
        vehicle->ApplySteeringRight(glfwGetKey(m_window.get(), GLFW_KEY_D) == GLFW_PRESS);
        vehicle->ApplySteeringLeft(glfwGetKey(m_window.get(), GLFW_KEY_A) == GLFW_PRESS);
    }
    if (glfwGetKey(m_window.get(), GLFW_KEY_R) == GLFW_PRESS)
    {
        ResetToVroad(m_nearestVroadID, 0.f);
    }
}