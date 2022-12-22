#pragma once

#include "../Util/ImageLoader.h"
#include "../Scene/Track.h"
#include "../Shaders/SkydomeShader.h"
#include "../Camera/BaseCamera.h"
#include "../Scene/Lights/GlobalLight.h"

const float SKYDOME_SCALE =10.0f;

class SkyRenderer
{
public:
    explicit SkyRenderer();
    ~SkyRenderer();
    void Render(const std::shared_ptr<BaseCamera> &camera, const std::shared_ptr<GlobalLight> &light, float elapsedTime);

private:
    // Load cloud, sun, moon and tint textures
    void _LoadAssets();
    SkydomeShader m_skydomeShader;
    // Sphere model for skydome
    CarModel m_skydomeModel;
    GLuint clouds1TextureID = 0, clouds2TextureID = 0, sunTextureID = 0, moonTextureID = 0, tintTextureID = 0, tint2TextureID = 0;
};