#include "TrackRenderer.h"

void TrackRenderer::Render(const std::vector<std::shared_ptr<CarAgent>> &racers,
                           const std::shared_ptr<BaseCamera> &camera,
                           GLuint trackTextureArrayID,
                           const std::vector<std::shared_ptr<Entity>> &visibleEntities,
                           const std::vector<shared_ptr<BaseLight>> &lights,
                           const ParamData &userParams,
                           GLuint depthTextureID,
                           float ambientFactor)
{
    m_trackShader.use();
    // This shader state doesnt change during a track renderpass
    m_trackShader.setClassic(userParams.useClassicGraphics);
    m_trackShader.loadProjectionViewMatrices(camera->projectionMatrix, camera->viewMatrix);
    m_trackShader.loadSpecular(userParams.trackSpecDamper, userParams.trackSpecReflectivity);
    m_trackShader.bindTextureArray(trackTextureArrayID);
    m_trackShader.loadShadowMapTexture(depthTextureID);
    m_trackShader.loadAmbientFactor(ambientFactor);
    m_trackShader.loadLights(lights);
    // m_trackShader.loadSpotlight(car->leftHeadlight);

    // TODO: Again, super silly.
    for (auto &light : lights)
    {
        if (light->type == LightType::GLOBAL_LIGHT)
        {
            m_trackShader.loadLightSpaceMatrix(std::static_pointer_cast<GlobalLight>(light)->lightSpaceMatrix);
        }
    }

    // Render the per-trackblock data
    for (auto &entity : visibleEntities)
    {
        m_trackShader.loadTransformMatrix(boost::get<TrackModel>(entity->raw).ModelMatrix);
        ///* Road signs flicker due to lack of culling.Enable culling specifically to render road signs.
        if((entity->type == EntityType::XOBJ))// && ((entity->flags) >> 4 & 0x7 == 5))
        {
            glEnable(GL_CULL_FACE);
            boost::get<TrackModel>(entity->raw).render();
            glDisable(GL_CULL_FACE);
        }
        else
        {
            boost::get<TrackModel>(entity->raw).render();
        }
        //*/
        //boost::get<TrackModel>(entity->raw).render();
    }

    m_trackShader.unbind();
    m_trackShader.HotReload();
}

void TrackRenderer::RenderLights(const std::shared_ptr<BaseCamera> &camera, const std::vector<shared_ptr<BaseLight>> &lights)
{
    //Disable depth write while rendering lights
    glDepthMask(GL_FALSE);
    m_billboardShader.use();

    for (auto &light : lights)
    {
        if (light->type == LightType::TRACK_LIGHT)
        {
            std::shared_ptr<TrackLight> trackLight = std::static_pointer_cast<TrackLight>(light);
            m_billboardShader.loadMatrices(camera->projectionMatrix, camera->viewMatrix);
            m_billboardShader.loadLight(trackLight);
            trackLight->model.render();
        }
    }
    m_billboardShader.unbind();
    m_billboardShader.HotReload();
    //Enable depth mask when done
    glDepthMask(GL_TRUE);
}

TrackRenderer::~TrackRenderer()
{
    // Cleanup VBOs and shaders
    m_trackShader.cleanup();
    m_billboardShader.cleanup();
}
