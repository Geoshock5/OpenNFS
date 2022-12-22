#pragma once

#include <LinearMath/btDefaultMotionState.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Vehicle/btVehicleRaycaster.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <btBulletDynamicsCommon.h>

#include "../RaceNet/RaceNet.h"
#include "../Scene/Lights/Spotlight.h"
#include "../Scene/Models/CarModel.h"
#include "../Util/ImageLoader.h"
#include "../Util/Utils.h"
#include "../Enums.h"
#include "al.h"
#include "alc.h"
#include "../Loaders/MusicLoader.h"

// Raycasting Data
enum RayDirection : uint8_t
{
    LEFT_RAY          = 0,
    FORWARD_LEFT_RAY  = 8,
    FORWARD_RAY       = 9,
    FORWARD_RIGHT_RAY = 10,
    RIGHT_RAY         = 18,
};

enum Gear
{
    R = 0,
    N = 1,
    F1 = 2,
    F2 = 3,
    F3 = 4,
    F4 = 5,
    F5 = 6,
    F6 = 7
};

constexpr uint8_t kNumRangefinders = 19;
constexpr float kFarDistance       = 5.f;
constexpr float kAngleBetweenRays  = 10.f;
constexpr float kCastDistance      = 1.f;

enum Wheels : uint8_t
{
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
};

struct VehicleProperties
{
    float mass;  // mass [kg](2)

    // Engine
    float maxEngineForce;   // Max engine force to apply (not an NFS field)
    float torqueCurve[41]; // torque curve (size 41) in 256rpm increments(10)
    float minRPM; // engine minimum rpm(12)
    float maxRPM; // engine redline in rpm(13)
    // float velocityCap;   // maximum velocity of car [m/s](14)
    float maxSpeed; // Max speed before stop applying engine force // top speed cap [m/s](15)

        // Brakes
    // bool useAbs;    // Uses Antilock Brake System(17)
    float maxBreakingForce; // Max breaking force // maximum braking deceleration(18)

    // Gearbox
    int numGearsMan;           // Total // manual number of gears (reverse + neutral + forward gears)(3)
    int numGearsAuto;           // Total // number of gears (automatic, r, n, forward)(75)
    int gearShiftDelay;        // Gear shift delay (ticks)
    // x-size array of values // shift blip in rpm (size 8)(5)
    // x-size array of values // brake blip in rpm (size 8)(6)
    float velToRpmMan[8]; // velocity to rpm ratio (size 8)(7)
    float velToRpmAuto[8]; // velocity to rpm ratio automatic(size 8)(76)
    float gearRatiosMan[8]; // gear ratios (size 8)(8)
    float gearRatiosAuto[8]; // gear ratios automatic (size 8)(77)
    float gearEfficiencyMan[8];  // gear efficiency (size 8)(9)
    float gearEfficiencyAuto[8]; // gear efficiency automatic (size 8)(78)
    float finalDriveMan; // final gear(11)
    float finalDriveAuto; // final gear automatic(79)
    float frontDriveRatio; // front drive ratio(16)
    
    // Steering
    bool absoluteSteer;      // Use absolute steering
    float steeringIncrement; // Steering speed
    float steeringClamp;     // Max steering angle

    // Wheel
    float wheelRadius;
    float wheelWidth;
    float wheelFriction;
    btScalar suspensionRestLength;

    // Suspension
    float suspensionStiffness;
    float suspensionDamping;
    float suspensionCompression;
    float rollInfluence; // Shift CoM

    // Visual
    glm::vec3 colour;
};

struct VehicleState
{
    // Engine
    float gEngineForce;   // Force to apply to engine
    float gBreakingForce; // Breaking force

    // Steering
    float gVehicleSteering;
    bool steerRight;
    bool steerLeft;
    bool isSteering;

    // Gearbox
    int currentGear=1;
    Gear eGear;
    float rpm;
};

struct RangefinderInfo
{
    float rangefinders[kNumRangefinders];
    glm::vec3 castPositions[kNumRangefinders];
    glm::vec3 upCastPosition, downCastPosition;
    float upDistance = 0.f, downDistance = 0.f;
};

struct RenderInfo
{
    bool isMultitexturedModel = false;
    GLuint textureID{};      // TGA texture ID
    GLuint textureArrayID{}; // Multitextured texture ID
};

class Car
{
public:
    explicit Car(const CarData& carData, NFSVer nfsVersion, const std::string& carID);
    Car(const CarData& carData, NFSVer nfsVersion, const std::string& carID, GLuint textureArrayID); // Multitextured car
    ~Car();
    void Update(btDynamicsWorld* dynamicsWorld);
    void SetPosition(glm::vec3 position, glm::quat orientation);
    void ApplyAccelerationForce(bool accelerate, bool reverse);
    void ApplyAccelerationForce(float accelerate, float reverse);
    void ApplyBrakingForce(bool apply);
    void ApplySteeringRight(bool apply);
    void ApplySteeringLeft(bool apply);
    void ApplyAbsoluteSteerAngle(float targetAngle);
    void ApplyAnalogSteering(float steeringAxis);
    void GearChange(bool shiftUp, bool shiftDown);
    float GetCarBodyOrientation();
    void StartSound();

    // Physics Engine registration
    void SetVehicle(btRaycastVehicle* vehicle)
    {
        m_vehicle = vehicle;
    }
    void SetRaycaster(btVehicleRaycaster* vehicleRayCaster)
    {
        m_vehicleRayCaster = vehicleRayCaster;
    }
    btRigidBody* GetVehicleRigidBody()
    {
        return m_carChassis;
    }
    btVehicleRaycaster* GetRaycaster()
    {
        return m_vehicleRayCaster;
    }
    btRaycastVehicle* GetVehicle()
    {
        return m_vehicle;
    }

    std::string name;
    std::string id;
    NFSVer tag;
    CarData assetData;

    // Car configuration data
    VehicleProperties vehicleProperties{};
    VehicleState vehicleState{};
    RangefinderInfo rangefinderInfo{};
    RenderInfo renderInfo{};

    // Meshes and Headlights
    Spotlight leftHeadlight{};
    Spotlight rightHeadlight{};
    std::vector<CarModel> miscModels;
    CarModel leftFrontWheelModel;
    CarModel rightFrontWheelModel;
    CarModel leftRearWheelModel;
    CarModel rightRearWheelModel;
    CarModel carBodyModel;

    // Wheel properties
    btRaycastVehicle::btVehicleTuning tuning;

private:
    void _UpdateMeshesToMatchPhysics();
    void _ApplyInputs();
    void _LoadTextures();
    void _LoadAudio();
    void _GenPhysicsModel();
    void _GenRaycasts(btDynamicsWorld* dynamicsWorld);
    void _SetModels(std::vector<CarModel> carModels);
    void _SetVehicleProperties();
    void _ReadNFS3CARP(std::fstream *infile, VehicleProperties *phys);

    // Gear logic
    bool justShifted = false;

    // Base Physics objects for car
    btDefaultMotionState* m_vehicleMotionState{}; // Retrieving vehicle location in world
    btRigidBody* m_carChassis{};
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btVehicleRaycaster* m_vehicleRayCaster{}; // Wheel simulation
    btRaycastVehicle* m_vehicle{};

    // Audio handling
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
    void* pData = NULL;
};
