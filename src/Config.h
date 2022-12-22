#pragma once

#include <imgui.h>
#include <cstdint>
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <boost/program_options.hpp>

#include "Enums.h"
#include "Util/Logger.h"

using namespace boost::program_options;

/* --------------- ONFS Compile time parameters here -----------------*/
/* Some graphics parameters can be found at file SHADER_PREAMBLE_PATH */
const std::string ONFS_VERSION    = "0.3";
const std::string ONFS_GL_VERSION = "330";

// ----- File paths for asset load/write -----
const std::string LOG_FILE_PATH        = "./";
const std::string SHADER_PREAMBLE_PATH = "../shaders/ShaderPreamble.h";

const std::string ASSET_PATH    = "./assets/";
const std::string CAR_PATH      = ASSET_PATH + "car/";
const std::string TRACK_PATH    = ASSET_PATH + "tracks/";
const std::string RESOURCE_PATH = "../resources/";

const std::string BEST_NETWORK_PATH = ASSET_PATH + "bestRacer.net";

const std::string NFS_2_TRACK_PATH = "/gamedata/tracks/pc/";
const std::string NFS_2_CAR_PATH   = "/gamedata/carmodel/pc/";

const std::string NFS_2_SE_TRACK_PATH = "/gamedata/tracks/se/";
const std::string NFS_2_SE_CAR_PATH   = "/gamedata/carmodel/pc/";

const std::string NFS_3_TRACK_PATH = "/gamedata/tracks/";
const std::string NFS_3_CAR_PATH   = "/gamedata/carmodel/";

const std::string NFS_4_TRACK_PATH = "/data/tracks/";
const std::string NFS_4_CAR_PATH   = "/data/cars/";

const std::string MCO_TRACK_PATH = "/data/tracks/";
const std::string MCO_CAR_PATH   = "/data/models/";

const std::string NFS_5_TRACK_PATH = "/gamedata/track/";
const std::string NFS_5_CAR_PATH   = "/gamedata/carmodel/";

// ----- Graphics -----
const uint16_t MAX_TEXTURE_ARRAY_SIZE = 512;
const uint32_t DEFAULT_X_RESOLUTION   = 1920;
const uint32_t DEFAULT_Y_RESOLUTION   = 1080;
const float DEFAULT_FOV               = 55.f;
// Shadow Map Resolution
const unsigned int SHADOW_WIDTH  = 2048; // Resolution of shadow map
const unsigned int SHADOW_HEIGHT = 2048;
// Lighting parameters - These should be adjusted in tandem with ShaderPreamble MAX_CONTRIB limits
const int LIGHTS_PER_NB_BLOCK         = 3; // Number of lights per neighbouring trackblock to contribute to current trackblock lighting
const int NEIGHBOUR_BLOCKS_FOR_LIGHTS = 1; // Number of neighbouring trackblocks to search for lights

// ----- Defaults -----
const std::string DEFAULT_CAR           = "corv";
const std::string DEFAULT_TRACK         = "trk003";
const std::string DEFAULT_CAR_NFS_VER   = ToString(NFS_3);
const std::string DEFAULT_TRACK_NFS_VER = ToString(NFS_3);
const int DEFAULT_NUM_RACERS            = 0;

/* --------------- ONFS Runtime parameters here -----------------*/
class Config
{
public:
    static Config& get()
    {
        static Config instance;
        return instance;
    }
    void ParseFile(std::ifstream& inStream);
    void InitFromCommandLine(int argc, char** argv);
    template <typename _T>
    _T getValue(const std::string& key)
    {
        return storedConfig[key].as<_T>();
    };
    // Better named parameters instead of using var_map with command-line arg name
    std::string car = DEFAULT_CAR, track = DEFAULT_TRACK;
    std::string carTag = DEFAULT_CAR_NFS_VER, trackTag = DEFAULT_TRACK_NFS_VER;
    uint16_t nRacers = DEFAULT_NUM_RACERS;
    /* -- Physics/AI Params -- */
    bool useFullVroad = true;
    bool sparkMode    = false;
    /* -- Render Params -- */
    bool vulkanRender = false;
    bool headless     = false;
    float fov         = DEFAULT_FOV;
    uint32_t resX = DEFAULT_X_RESOLUTION, resY = DEFAULT_Y_RESOLUTION;
    /* -- Training Params -- */
    bool trainingMode     = false;
    uint16_t nGenerations = 0;
    uint32_t nTicks;
    /* -- Tool Params -- */
    bool renameAssets = false;

private:
    Config() = default;
    Config(const Config&);
    Config& operator=(const Config&);
    variables_map storedConfig;
};

struct ParamData
{
    float timeScaleFactor       = 0.1f;
    ImVec4 sunAttenuation       = ImVec4(0.f, 0.f, 0.f, 1.0f);
    float trackSpecReflectivity = 1;
    float nearPlane             = 160.f;
    float farPlane              = 300.f;
    float trackSpecDamper       = 10;
    int blockDrawDistance       = 15;
    bool physicsDebugView       = false;
    bool drawHermiteFrustum     = false;
    bool drawTrackAABB          = false;
    bool useClassicGraphics     = true;
    bool attachCamToHermite     = false;
    bool useNbData              = true;
    bool attachCamToCar         = true;
    bool frustumCull            = false;
    bool drawVroad              = false;
    bool drawCAN                = false;
    bool drawRaycast            = false;
    bool simulateCars           = true;
};

struct AssetData
{
    NFSVer carTag;
    std::string car;
    NFSVer trackTag;
    std::string track;
};

struct NfsAssetList
{
    NFSVer tag;
    std::vector<std::string> tracks;
    std::vector<std::string> cars;
};

struct JoyState
{
    /* -- Joystick state -- */
    int joynum      = 0;
    int joy0axcount = NULL;
    const float* joy0axes;
    int joy0butcount = NULL;
    const unsigned char* joy0buttons;
};