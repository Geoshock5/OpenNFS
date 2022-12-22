#include "Car.h"

#include "../Scene/Entity.h"

// Forward casts should extend further than L/R
constexpr float kCastDistances[kNumRangefinders] = {
  1.f, 1.f, 1.f, 1.f, 1.f, 1.5f, 2.f, 3.f, 5.f, 5.f, 5.f, 3.f, 2.f, 2.f, 1.5f, 1.f, 1.f, 1.f, 1.f,
};

Car::Car(const CarData &carData, NFSVer nfsVersion, const std::string &carID, GLuint textureArrayID) : Car(carData, nfsVersion, carID)
{
    renderInfo.textureArrayID       = textureArrayID;
    renderInfo.isMultitexturedModel = true;
}

Car::Car(const CarData &carData, NFSVer nfsVersion, const std::string &carID) : id(carID), assetData(carData), tag(nfsVersion)
{
    // Vehicle names are only encoded in mesh Asset files for NFS 3 and 4, we must rely upon part of the filename for other titles
    name = carData.carName.empty() ? id : carData.carName;

    // Load in vehicle texture data to OpenGL
    if (!Config::get().vulkanRender)
    {
        this->_LoadTextures();
    }

    // Configure performance data
    this->_SetVehicleProperties();

    // Map mesh data to car data
    this->_SetModels(assetData.meshes);

    // Build the Bullet physics representation of the vehicle
    this->_GenPhysicsModel();

    // Load audio
    this->_LoadAudio();
}

Car::~Car()
{
    // Clean up audio
    alDeleteBuffers(1, &uiBuffer);
    alDeleteSources(1, &uiSource);
    
    // Clean up the vehicle meshes
    leftFrontWheelModel.destroy();
    rightFrontWheelModel.destroy();
    leftRearWheelModel.destroy();
    rightRearWheelModel.destroy();
    carBodyModel.destroy();
    for (auto &miscModel : miscModels)
    {
        miscModel.destroy();
    }
    // And bullet collision shapes on heap
    m_collisionShapes.clear();
    // And the loaded GL textures
    if (renderInfo.isMultitexturedModel)
    {
        // TODO: Store number of textures so can pass correct parameter here
        glDeleteTextures(1, &renderInfo.textureArrayID);
    }
    else
    {
        glDeleteTextures(1, &renderInfo.textureID);
    }
}

void Car::StartSound()
{
    // Attach source to buffer, queue and play
    alSourcei(uiSource, AL_BUFFER, uiBuffer);
    alSourcei(uiSource, AL_LOOPING, AL_TRUE);
    // alSourceQueueBuffers(uiSource, 1, &uiBuffer);
    alSourcePlay(uiSource);
}

void Car::Update(btDynamicsWorld *dynamicsWorld)
{
    // Update car
    this->_UpdateMeshesToMatchPhysics();
    // Apply user input
    this->_ApplyInputs();
    // Update raycasts
    this->_GenRaycasts(dynamicsWorld);
    
    // Update engine note
    alSourcef(uiSource, AL_PITCH, vehicleState.rpm / 2000.f);
}

void Car::ApplyAccelerationForce(bool accelerate, bool reverse)
{
    if (accelerate)
    {
        if (m_vehicle->getCurrentSpeedKmHour() < vehicleProperties.maxSpeed * 3.6)
        {
            vehicleState.gEngineForce   = vehicleProperties.maxEngineForce;
            vehicleState.gBreakingForce = 0.f;
        }
        else
        {
            vehicleState.gEngineForce = 0.f;
        }
    }
    else if (reverse)
    {
        vehicleState.gEngineForce   = -vehicleProperties.maxEngineForce;
        vehicleState.gBreakingForce = 0.f;
    }
    else
    {
        vehicleState.gEngineForce = 0.f;
    }
}

void Car::ApplyAccelerationForce(float accelerate, float reverse)
{
    float rpm = m_vehicle->getCurrentSpeedKmHour() / 3.6 / (vehicleProperties.wheelRadius * 2 * glm::pi<float>()) *
      vehicleProperties.finalDriveMan * vehicleProperties.gearRatiosMan[vehicleState.currentGear] * 60;

    vehicleState.rpm = glm::clamp<float>(rpm, vehicleProperties.minRPM, vehicleProperties.maxRPM);
    if (accelerate)
    {
        if (m_vehicle->getCurrentSpeedKmHour() < vehicleProperties.maxSpeed * 3.6)
        {
            int direction   = (vehicleState.currentGear == 0) ? -1 : 1;            
            int torqueIndex        = glm::floor(vehicleState.rpm / 256.f);
            vehicleState.gEngineForce = direction * accelerate * vehicleProperties.torqueCurve[torqueIndex] * vehicleProperties.gearRatiosMan[vehicleState.currentGear] * vehicleProperties.finalDriveMan / vehicleProperties.wheelRadius;
            //vehicleState.gEngineForce   = vehicleProperties.maxEngineForce * accelerate;
            vehicleState.gBreakingForce = vehicleProperties.maxBreakingForce * reverse;
        }
        else
        {
            vehicleState.gEngineForce = 0.f;
            vehicleState.gBreakingForce = vehicleProperties.maxBreakingForce * reverse;
        }
    }
    /* else if (reverse)
    {
        vehicleState.gEngineForce   = -vehicleProperties.maxEngineForce * reverse;
        vehicleState.gBreakingForce = 0.f;
    }
    else
    {
        vehicleState.gEngineForce = 0.f;
    }*/
}

void Car::ApplySteeringRight(bool apply)
{
    vehicleState.steerRight = apply;
}

void Car::ApplySteeringLeft(bool apply)
{
    vehicleState.steerLeft = apply;
}

void Car::ApplyAbsoluteSteerAngle(float targetAngle)
{
    // Allow the update() method to directly utilise this targetAngle value
    vehicleProperties.absoluteSteer = true;
    // NN will always produce positive value, drop 0.5f from 0 -> 1 step output to allow -0.5 to 0.5
    float finalSteering = targetAngle; // - 0.5f;
    // Clamp value within steering extents
    vehicleState.gVehicleSteering = std::max(-vehicleProperties.steeringClamp, std::min(finalSteering, vehicleProperties.steeringClamp));
}

void Car::ApplyAnalogSteering(float steeringAxis)
{
    vehicleProperties.absoluteSteer = true;
    vehicleState.gVehicleSteering = vehicleProperties.steeringClamp * -steeringAxis;
}

void Car::ApplyBrakingForce(bool apply)
{
    if (apply)
    {
        vehicleState.gBreakingForce = vehicleProperties.maxBreakingForce;
    }
    else
    {
        vehicleState.gBreakingForce = 0.f;
    }
}

void Car::GearChange(bool shiftUp, bool shiftDown)
{
    if (!justShifted && shiftUp && (vehicleState.currentGear < vehicleProperties.numGearsMan))
    {
        justShifted = true;
        ++vehicleState.currentGear;
    }
    if (!justShifted && shiftDown && (vehicleState.currentGear > 0))
    {
        justShifted = true;
        --vehicleState.currentGear;
    }
    else if (!shiftUp && !shiftDown && justShifted)
    {
        justShifted = false;
    }
    vehicleState.eGear = (Gear)vehicleState.currentGear;
}

void Car::SetPosition(glm::vec3 position, glm::quat orientation)
{
    m_carChassis->clearForces();
    m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
    m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
    m_vehicle->resetSuspension();
    for (int wheelIdx = 0; wheelIdx < m_vehicle->getNumWheels(); ++wheelIdx)
    {
        // Synchronize the wheels with the (interpolated) chassis worldt ransform
        m_vehicle->updateWheelTransform(wheelIdx, true);
    }

    // Create bullet transform of new positional and directional data
    btTransform positionTransform = Utils::MakeTransform(position, orientation);
    m_carChassis->setWorldTransform(positionTransform);

    // Update mesh positions to match new chassis transform
    this->_UpdateMeshesToMatchPhysics();
}

float Car::GetCarBodyOrientation()
{
    glm::quat orientation = carBodyModel.orientation;
    return glm::degrees(atan2(2 * orientation.y * orientation.w - 2 * orientation.x * orientation.z, 1 - 2 * orientation.y * orientation.y - 2 * orientation.z * orientation.z));
}

void Car::_UpdateMeshesToMatchPhysics()
{
    btTransform trans;
    m_vehicleMotionState->getWorldTransform(trans);
    carBodyModel.position    = Utils::bulletToGlm(trans.getOrigin()) + (carBodyModel.initialPosition * glm::inverse(Utils::bulletToGlm(trans.getRotation())));
    carBodyModel.orientation = Utils::bulletToGlm(trans.getRotation());
    carBodyModel.update();

    // Might as well apply the body transform to the Miscellaneous models
    for (auto &miscModel : miscModels)
    {
        miscModel.position    = Utils::bulletToGlm(trans.getOrigin()) + (miscModel.initialPosition * glm::inverse(Utils::bulletToGlm(trans.getRotation())));
        miscModel.orientation = Utils::bulletToGlm(trans.getRotation());
        miscModel.update();
    }

    // Update headlight direction vectors to match car body
    leftHeadlight.direction  = Utils::bulletToGlm(m_vehicle->getForwardVector());
    rightHeadlight.direction = Utils::bulletToGlm(m_vehicle->getForwardVector());
    leftHeadlight.position   = Utils::bulletToGlm(trans.getOrigin()) + (leftHeadlight.initialPosition * glm::inverse(Utils::bulletToGlm(trans.getRotation())));
    rightHeadlight.position  = Utils::bulletToGlm(trans.getOrigin()) + (rightHeadlight.initialPosition * glm::inverse(Utils::bulletToGlm(trans.getRotation())));

    // Lets go update wheel geometry positions based on physics feedback
    for (int wheelIdx = 0; wheelIdx < m_vehicle->getNumWheels(); ++wheelIdx)
    {
        m_vehicle->updateWheelTransform(wheelIdx, true);
        trans = m_vehicle->getWheelInfo(wheelIdx).m_worldTransform;
        switch (wheelIdx)
        {
        case Wheels::FRONT_LEFT:
            leftFrontWheelModel.position    = Utils::bulletToGlm(trans.getOrigin());
            leftFrontWheelModel.orientation = Utils::bulletToGlm(trans.getRotation());
            leftFrontWheelModel.update();
            break;
        case Wheels::FRONT_RIGHT:
            rightFrontWheelModel.position    = Utils::bulletToGlm(trans.getOrigin());
            rightFrontWheelModel.orientation = Utils::bulletToGlm(trans.getRotation());
            rightFrontWheelModel.update();
            break;
        case Wheels::REAR_LEFT:
            leftRearWheelModel.position    = Utils::bulletToGlm(trans.getOrigin());
            leftRearWheelModel.orientation = Utils::bulletToGlm(trans.getRotation());
            leftRearWheelModel.update();
            break;
        case Wheels::REAR_RIGHT:
            rightRearWheelModel.position    = Utils::bulletToGlm(trans.getOrigin());
            rightRearWheelModel.orientation = Utils::bulletToGlm(trans.getRotation());
            rightRearWheelModel.update();
            break;
        default:
            ASSERT(false, "More than 4 wheels currently unsupported");
            break;
        }
    }
}

void Car::_ApplyInputs()
{
    if (!vehicleProperties.absoluteSteer)
    {
        // update front wheels steering value
        if (vehicleState.steerRight)
        {
            vehicleState.gVehicleSteering -= vehicleProperties.steeringIncrement;
            if (vehicleState.gVehicleSteering < -vehicleProperties.steeringClamp)
            {
                vehicleState.gVehicleSteering = -vehicleProperties.steeringClamp;
            }
        }
        else if (vehicleState.steerLeft)
        {
            vehicleState.gVehicleSteering += vehicleProperties.steeringIncrement;
            if (vehicleState.gVehicleSteering > vehicleProperties.steeringClamp)
            {
                vehicleState.gVehicleSteering = vehicleProperties.steeringClamp;
            }
        }
        else
        {
            if (vehicleState.gVehicleSteering > 0)
            {
                vehicleState.gVehicleSteering -= vehicleProperties.steeringIncrement;
            }
            else if (vehicleState.gVehicleSteering < 0)
            {
                vehicleState.gVehicleSteering += vehicleProperties.steeringIncrement;
            }
        }
    }

    // Set front wheels steering value
    m_vehicle->setSteeringValue(vehicleState.gVehicleSteering, Wheels::FRONT_LEFT);
    m_vehicle->setSteeringValue(vehicleState.gVehicleSteering, Wheels::FRONT_RIGHT);
    // Apply engine force to rear wheels (RWD only)
    m_vehicle->applyEngineForce(vehicleState.gEngineForce, Wheels::REAR_LEFT);
    m_vehicle->setBrake(vehicleState.gBreakingForce, Wheels::REAR_LEFT);
    m_vehicle->applyEngineForce(vehicleState.gEngineForce, Wheels::REAR_RIGHT);
    m_vehicle->setBrake(vehicleState.gBreakingForce, Wheels::REAR_RIGHT);
}

void Car::_LoadTextures()
{
    std::stringstream carTexturePath;
    int width, height;
    carTexturePath << CAR_PATH << ToString(tag) << "/" << id;

    if (tag == NFS_3 || tag == NFS_4)
    {
        carTexturePath << "/car00.tga";
        renderInfo.textureID = ImageLoader::LoadImage(carTexturePath.str(), &width, &height, GL_CLAMP_TO_BORDER, GL_LINEAR_MIPMAP_LINEAR);
    }
    else if (tag == MCO)
    {
        std::stringstream car_alpha_texture_path;
        carTexturePath << "/Textures/0000.BMP";
        car_alpha_texture_path << CAR_PATH << ToString(tag) << "/" << id << "/Textures/0000-a.BMP";
        GLubyte *imageData;
        if (ImageLoader::LoadBmpWithAlpha(carTexturePath.str().c_str(), car_alpha_texture_path.str().c_str(), &imageData, &width, &height))
        {
            glGenTextures(1, &renderInfo.textureID);
            glBindTexture(GL_TEXTURE_2D, renderInfo.textureID);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData);
            glGenerateMipmap(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
    }
}

void Car::_LoadAudio()
{
    // Set up and load sounds
    BnkLoader bnkFile;
    std::vector<AudioBuffer> horn = bnkFile.LoadBnk("assets/car/NFS_3/corv/car.bnk");
    LOG(INFO) << "Buffer has " << horn[0].GetHeaderPtr()->dwNumSamples << " samples of data. Buffer size " << sizeof(*horn[0].GetBufPtr()) << " bytes";

    // Generate buffers and sources
    alGenBuffers(1, &uiBuffer);
    alGenSources(1, &uiSource);

    // Add file to buffer
    alBufferData(
      uiBuffer, AL_FORMAT_MONO16, horn[0].GetBufPtr(), horn[0].GetHeaderPtr()->dwNumSamples * horn[0].GetHeaderPtr()->dwBytesPerSample, horn[0].GetHeaderPtr()->dwSampleRate);
}

void Car::_GenPhysicsModel()
{
    // Get the size of a wheel
    DimensionData wheelDimensions = Utils::GenDimensions(leftFrontWheelModel.m_vertices);
    glm::vec3 wheelSize           = glm::vec3((wheelDimensions.maxVertex.x - wheelDimensions.minVertex.x) / 2,
                                    (wheelDimensions.maxVertex.y - wheelDimensions.minVertex.y) / 2,
                                    (wheelDimensions.maxVertex.z - wheelDimensions.minVertex.z) / 2);
    vehicleProperties.wheelRadius = wheelSize.z;
    vehicleProperties.wheelWidth  = wheelSize.x;

    LOG(INFO) << "PHYS: Calculated wheelbase " << glm::distance(leftFrontWheelModel.position, leftRearWheelModel.position);
    LOG(INFO) << "PHYS: Calculated radius " << vehicleProperties.wheelRadius;

    // Generate the chassis collision mesh
    DimensionData chassisDimensions = Utils::GenDimensions(carBodyModel.m_vertices);
    // Drop size of car chassis vertically to avoid colliding with ground on suspension compression
    chassisDimensions.minVertex.y += 0.04f;
    btCollisionShape *chassisShape = new btBoxShape(Utils::glmToBullet((chassisDimensions.maxVertex - chassisDimensions.minVertex) / 2.f));
    m_collisionShapes.push_back(chassisShape);

    auto *compound = new btCompoundShape();
    btTransform localTrans;
    localTrans.setIdentity();

    // Shift center of Mass
    float centerOfMassShiftY;
    switch (tag)
    {
    case NFS_3:
    case NFS_4:
    case MCO:
        centerOfMassShiftY = 0.f;
        break;
    case NFS_3_PS1:
        centerOfMassShiftY = 0.1f;
        break;
    default:
        centerOfMassShiftY = 0.05f;
        break;
    }

    localTrans.setOrigin(btVector3(0.0, centerOfMassShiftY, 0));
    compound->addChildShape(localTrans, chassisShape);

    btVector3 localInertia(0, 0, 0);
    compound->calculateLocalInertia(vehicleProperties.mass, localInertia);
    m_collisionShapes.push_back(compound);

    // Set initial location of vehicle in the world
    m_vehicleMotionState = new btDefaultMotionState(btTransform(btQuaternion(Utils::glmToBullet(carBodyModel.orientation)), Utils::glmToBullet(carBodyModel.position)));
    btRigidBody::btRigidBodyConstructionInfo cInfo(vehicleProperties.mass, m_vehicleMotionState, compound, localInertia);
    m_carChassis = new btRigidBody(cInfo);

    // Abuse Entity system with a dummy entity that wraps the car pointer instead of a GL mesh
    m_carChassis->setUserPointer(new Entity(-1, -1, tag, EntityType::CAR, this, 0));
    m_carChassis->setDamping(0.2f, 0.2f);
    m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
    m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
}

void Car::_GenRaycasts(btDynamicsWorld *dynamicsWorld)
{
    btTransform trans;
    m_vehicleMotionState->getWorldTransform(trans);
    glm::vec3 carBodyPosition = Utils::bulletToGlm(trans.getOrigin());

    // Get base vectors
    glm::vec3 carUp      = carBodyModel.ModelMatrix * glm::vec4(0, 1, 0, 0);
    glm::vec3 carForward = Utils::bulletToGlm(m_vehicle->getForwardVector());

    btCollisionWorld::ClosestRayResultCallback *rayCallbacks[kNumRangefinders];
    glm::vec3 castVectors[kNumRangefinders];

    for (uint8_t rangeIdx = 0; rangeIdx < kNumRangefinders; ++rangeIdx)
    {
        // Calculate base vector from -90 + (rangeIdx * kAngleBetweenRays) from car forward vector
        castVectors[rangeIdx] = carForward * glm::normalize(glm::quat(glm::vec3(0, glm::radians(-90.f + (rangeIdx * kAngleBetweenRays)), 0)));
        // Calculate where the ray will cast out to
        rangefinderInfo.castPositions[rangeIdx] = carBodyPosition + (castVectors[rangeIdx] * kCastDistances[rangeIdx]);
        rayCallbacks[rangeIdx] = new btCollisionWorld::ClosestRayResultCallback(Utils::glmToBullet(carBodyPosition), Utils::glmToBullet(rangefinderInfo.castPositions[rangeIdx]));
        // Don't Raycast against other opponents for now. Ghost through them. Only interested in VROAD edge.
        rayCallbacks[rangeIdx]->m_collisionFilterMask = COL_TRACK;
        // Perform the raycast
        dynamicsWorld->rayTest(Utils::glmToBullet(carBodyPosition), Utils::glmToBullet(rangefinderInfo.castPositions[rangeIdx]), *rayCallbacks[rangeIdx]);
        // Check whether we hit anything
        if (rayCallbacks[rangeIdx]->hasHit())
        {
            rangefinderInfo.rangefinders[rangeIdx] = glm::distance(carBodyPosition, Utils::bulletToGlm(rayCallbacks[rangeIdx]->m_hitPointWorld));
        }
        else
        {
            rangefinderInfo.rangefinders[rangeIdx] = kFarDistance;
        }
        delete rayCallbacks[rangeIdx];
    }
    rangefinderInfo.upCastPosition   = (carBodyPosition + (carUp * kCastDistance));
    rangefinderInfo.downCastPosition = (carBodyPosition + (-carUp * kCastDistance));

    btCollisionWorld::ClosestRayResultCallback upRayCallback(Utils::glmToBullet(carBodyPosition), Utils::glmToBullet(rangefinderInfo.upCastPosition));
    btCollisionWorld::ClosestRayResultCallback downRayCallback(Utils::glmToBullet(carBodyPosition), Utils::glmToBullet(rangefinderInfo.downCastPosition));
    // Up raycast is used to check for flip over, and also whether inside VROAD
    upRayCallback.m_collisionFilterMask = downRayCallback.m_collisionFilterMask = COL_TRACK | COL_VROAD_CEIL;
    dynamicsWorld->rayTest(Utils::glmToBullet(carBodyPosition), Utils::glmToBullet(rangefinderInfo.upCastPosition), upRayCallback);
    dynamicsWorld->rayTest(Utils::glmToBullet(carBodyPosition), Utils::glmToBullet(rangefinderInfo.downCastPosition), downRayCallback);

    if (upRayCallback.hasHit())
    {
        rangefinderInfo.upDistance = glm::distance(carBodyPosition, Utils::bulletToGlm(upRayCallback.m_hitPointWorld));
    }
    else
    {
        rangefinderInfo.upDistance = kFarDistance;
    }
    if (downRayCallback.hasHit())
    {
        rangefinderInfo.downDistance = glm::distance(carBodyPosition, Utils::bulletToGlm(downRayCallback.m_hitPointWorld));
    }
    else
    {
        rangefinderInfo.downDistance = kFarDistance;
    }
}

// Take the list of Meshes returned by the car loader, and pull the High res wheels and body out for physics to manipulate
void Car::_SetModels(std::vector<CarModel> carModels)
{
    switch (tag)
    {
    case NFS_1:
        break;
    case NFS_2_PS1:
    case NFS_2_SE:
    case NFS_2:
    case NFS_3_PS1:
    {
        if (carModels.size() < 3)
        {
            carModels[0].enable();
            leftFrontWheelModel  = carModels[0];
            rightFrontWheelModel = carModels[0];
            leftRearWheelModel   = carModels[0];
            rightRearWheelModel  = carModels[0];
            carModels[1].enable();
            carBodyModel = carModels[1];
        }
        else
        {
            for (auto &carModel : carModels)
            {
                if (carModel.m_name == "High Main Body Part")
                {
                    carModel.enable();
                    carBodyModel = carModel;
                }
                else if (carModel.m_name.find("High Front Left Wheel Part") != std::string::npos)
                {
                    carModel.enable();
                    leftFrontWheelModel = carModel;
                }
                else if (carModel.m_name.find("High Front Right Wheel Part") != std::string::npos)
                {
                    carModel.enable();
                    rightFrontWheelModel = carModel;
                }
                else if (carModel.m_name.find("High Rear Left Wheel Part") != std::string::npos)
                {
                    carModel.enable();
                    leftRearWheelModel = carModel;
                }
                else if (carModel.m_name.find("High Rear Right Wheel Part") != std::string::npos)
                {
                    carModel.enable();
                    rightRearWheelModel = carModel;
                }
                else if (carModel.m_name.find("High") != std::string::npos)
                { // Everything with "High" in the name is an extra body part, enable it
                    carModel.enable();
                    miscModels.emplace_back(carModel);
                }
                else
                {
                    miscModels.emplace_back(carModel);
                }
            }
        }
    }
    break;
    case NFS_3:
    {
        carBodyModel = carModels[0];
        carBodyModel.enable();
        leftFrontWheelModel = carModels[1];
        leftFrontWheelModel.enable();
        rightFrontWheelModel = carModels[2];
        rightFrontWheelModel.enable();
        leftRearWheelModel = carModels[3];
        leftRearWheelModel.enable();
        rightRearWheelModel = carModels[4];
        rightRearWheelModel.enable();
        if (carModels.size() >= 5)
        {
            for (size_t partIdx = 5; partIdx < carModels.size(); ++partIdx)
            {
                miscModels.emplace_back(carModels[partIdx]);
            }
        }
    }
    break;
    case NFS_4:
        for (auto &carModel : carModels)
        {
            if (carModel.m_name == ":HB")
            {
                carModel.enable();
                carBodyModel = carModel;
            }
            else if (carModel.m_name == ":HLRW")
            {
                carModel.enable();
                leftRearWheelModel = carModel;
            }
            else if (carModel.m_name == ":HLFW")
            {
                carModel.enable();
                leftFrontWheelModel = carModel;
            }
            else if (carModel.m_name == ":HRRW")
            {
                carModel.enable();
                rightRearWheelModel = carModel;
            }
            else if (carModel.m_name == ":HRFW")
            {
                carModel.enable();
                rightFrontWheelModel = carModel;
            }
            else if (carModel.m_name.find('O') != std::string::npos)
            {
                carModel.enable();
                miscModels.emplace_back(carModel);
            }
            else
            {
                miscModels.emplace_back(carModel);
            }
        }
        break;
    case NFS_4_PS1:
        for (auto &carModel : carModels)
        {
            if (carModel.m_name.find("Right Body High") != std::string::npos)
            {
                carModel.enable();
                carBodyModel = carModel;
            }
            else if (carModel.m_name.find("Rear Left Wheel") != std::string::npos)
            {
                carModel.enable();
                leftRearWheelModel = carModel;
            }
            else if (carModel.m_name.find("Front Left Tire") != std::string::npos)
            {
                carModel.enable();
                leftFrontWheelModel = carModel;
            }
            else if (carModel.m_name.find("Rear Right Wheel") != std::string::npos)
            {
                carModel.enable();
                rightRearWheelModel = carModel;
            }
            else if (carModel.m_name.find("Front Right Tire") != std::string::npos)
            {
                carModel.enable();
                rightFrontWheelModel = carModel;
            }
            else if (carModel.m_name.find("Left Body High") != std::string::npos)
            {
                carModel.enable();
                miscModels.emplace_back(carModel);
            }
            else
            {
                miscModels.emplace_back(carModel);
            }
        }
        break;
    case MCO:
        for (auto &carModel : carModels)
        {
            if (carModel.m_name == ":Hbody")
            {
                carModel.enable();
                carBodyModel = carModel;
            }
            else if (carModel.m_name == ":PPLRwheel")
            {
                carModel.enable();
                leftRearWheelModel = carModel;
            }
            else if (carModel.m_name == ":PPLFwheel")
            {
                carModel.enable();
                leftFrontWheelModel = carModel;
            }
            else if (carModel.m_name == ":PPRRwheel")
            {
                carModel.enable();
                rightRearWheelModel = carModel;
            }
            else if (carModel.m_name == ":PPRFwheel")
            {
                carModel.enable();
                rightFrontWheelModel = carModel;
            }
            else if (carModel.m_name.find(":H") != std::string::npos)
            {
                carModel.enable();
                miscModels.emplace_back(carModel);
            }
            else
            {
                miscModels.emplace_back(carModel);
            }
        }
        break;
    case UNKNOWN:
        break;
    case NFS_5:
    {
        for (auto &carModel : carModels)
        {
            if (carModel.m_name.find("Body_ig1") != std::string::npos)
            {
                carModel.enable();
                carBodyModel = carModel;
            }
            else if (carModel.m_name.find("WheelFront_fe1") != std::string::npos)
            {
                carModel.enable();
                leftFrontWheelModel = carModel;
            }
            else if (carModel.m_name.find("WheelFront_ig1") != std::string::npos)
            {
                carModel.enable();
                rightFrontWheelModel = carModel;
            }
            else if (carModel.m_name.find("WheelRear_fe1") != std::string::npos)
            {
                carModel.enable();
                leftRearWheelModel = carModel;
            }
            else if (carModel.m_name.find("WheelRear_ig1") != std::string::npos)
            {
                carModel.enable();
                rightRearWheelModel = carModel;
            }
            else
            {
                // Enable all High LOD ig1 models
                carModel.enabled = ((carModel.m_name.find("ig1") != std::string::npos) && (carModel.m_name.find("Shadow") == std::string::npos));
                miscModels.emplace_back(carModel);
            }
        }
    }
    break;
    }

    // Go find headlight position data inside dummies
    if (tag == NFS_3 || tag == NFS_4)
    {
        for (auto &dummy : assetData.dummies)
        {
            if (dummy.name.find("HFLO") != std::string::npos)
            {
                leftHeadlight.cutOff   = glm::cos(glm::radians(12.5f));
                leftHeadlight.position = leftHeadlight.initialPosition = dummy.position;
                leftHeadlight.colour                                   = glm::vec3(1, 1, 1);
            }
            if (dummy.name.find("HFRE") != std::string::npos)
            {
                rightHeadlight.cutOff   = glm::cos(glm::radians(12.5f));
                rightHeadlight.position = rightHeadlight.initialPosition = dummy.position;
                rightHeadlight.colour                                    = glm::vec3(1, 1, 1);
            }
            // TRLN, TRRN for tail lights
        }
    }
    else
    {
        leftHeadlight.cutOff = rightHeadlight.cutOff = glm::cos(glm::radians(12.5f));
        leftHeadlight.position = rightHeadlight.position = carBodyModel.position;
        leftHeadlight.colour = rightHeadlight.colour = glm::vec3(1, 1, 1);
    }
}

void Car::_SetVehicleProperties()
{
    // TODO: Load these from Carp.txt
    vehicleProperties.mass                  = 1750.f;
    vehicleProperties.maxSpeed              = 20.f;
    vehicleProperties.maxEngineForce        = 30000.f; // Was 3000;
    vehicleProperties.maxBreakingForce      = 1000.f;
    vehicleProperties.suspensionRestLength  = btScalar(0.020);
    vehicleProperties.suspensionStiffness   = 750.f;
    vehicleProperties.suspensionDamping     = 200.f;
    vehicleProperties.suspensionCompression = 500.4f;
    vehicleProperties.wheelFriction         = 0.45f;
    vehicleProperties.rollInfluence         = 0.04f;
    vehicleProperties.steeringIncrement     = 0.01f;
    vehicleProperties.steeringClamp         = 0.15f;
    vehicleProperties.absoluteSteer         = false;
    
    // WIP: Load from Carp.txt
    std::fstream carPhysicsFile;
    std::stringstream carPhysicsPath;
    carPhysicsPath << CAR_PATH << ToString(tag) << "/" << id;

    if (tag == NFS_3 || tag == NFS_4)
    {
        carPhysicsPath << "/carp.txt";
        carPhysicsFile.open(carPhysicsPath.str(), std::ios::in);
        if (carPhysicsFile.is_open())
        {
            _ReadNFS3CARP(&carPhysicsFile, &vehicleProperties);
        }
        carPhysicsFile.close();
    }
    else if (tag == MCO)
    {
        // Not done yet
    }
    
    
    // Set car colour
    if (!assetData.colours.empty())
    {
        int randomColourIdx      = (int) Utils::RandomFloat(0.f, (float) assetData.colours.size());
        vehicleProperties.colour = assetData.colours[randomColourIdx].colour;
    }
    else
    {
        vehicleProperties.colour = glm::vec3(Utils::RandomFloat(0.f, 1.f), Utils::RandomFloat(0.f, 1.f), Utils::RandomFloat(0.f, 1.f));
    }

    // State
    vehicleState.gEngineForce     = 0.f;
    vehicleState.gBreakingForce   = 100.f;
    vehicleState.gVehicleSteering = 0.f;
    vehicleState.steerRight = vehicleState.steerLeft = false;
}

void Car::_ReadNFS3CARP(std::fstream* infile, VehicleProperties* phys)
{
    std::string line;
    size_t start;
    size_t end;
    uint8_t lineID = 0;
    
    std::stringstream str;
    std::string word = "";
    bool inFile      = true;

    while (std::getline(*infile, line)&&inFile)
    {      
        start = line.find_last_of("(");
        end   = line.find_last_of(")");
        str.seekg(0);
        
        lineID = 0;
        for (size_t i = start+1; i < end; i++)
        {
            char s;
            line.copy(&s, 1, i);
            lineID = ((lineID << 1) + (lineID << 3)) + (s-'0');
        }

        //LOG(INFO) << "Physics line: " << line;
        //LOG(INFO) << "Start : " << start << ", end " << end << ". Switch on " << (int)lineID;
        
        switch (lineID)
        {
        case (2):
            std::getline(*infile, line);
            phys->mass = std::stof(line);
            LOG(INFO) << "Vehicle mass set to " << phys->mass << "kg";
            break;
        case (3):
            std::getline(*infile, line);
            phys->numGearsMan = std::stoi(line);
            LOG(INFO) << "Vehicle manual gearbox set to " << phys->numGearsMan << " gears";
            break;
        case (75):
            std::getline(*infile, line);
            phys->numGearsAuto = std::stoi(line);
            LOG(INFO) << "Vehicle auto gearbox set to " << phys->numGearsAuto << " gears";
            break;
        case (7) :
            std::getline(*infile, line);
            str.str(line);
            for (size_t i = 0; i < 8; i++)
            {
                if (std::getline(str, word, ','))
                {
                    // LOG(INFO) << "Torque entry: " << word;
                    phys->velToRpmMan[i] = stof(word);
                }
            }
            LOG(INFO) << "Vehicle vel-to-RPM manual ratios set.";
            break;
        case (8):
            std::getline(*infile, line);
            str.str(line);
            for (size_t i = 0; i < 8; i++)
            {
                if (std::getline(str, word, ','))
                {
                    // LOG(INFO) << "Torque entry: " << word;
                    phys->gearRatiosMan[i] = stof(word);
                }
            }
            LOG(INFO) << "Vehicle manual gear ratios set.";
            break;
        case (10):
            std::getline(*infile, line);
            str.str(line);
            LOG(INFO) << "Line input " << line;
            for (size_t i = 0; i < 41; i++)
            {
                if (std::getline(str, word, ','))
                {
                    //LOG(INFO) << "Torque entry: " << word;
                    phys->torqueCurve[i] = stof(word);
                }
            }
            LOG(INFO) << "Vehicle torque curve set.";
            break;
        case (11):
            std::getline(*infile, line);
            phys->finalDriveMan = std::stof(line);
            LOG(INFO) << "Vehicle final drive manual set to " << phys->finalDriveMan;
            break;
        case (79):
            std::getline(*infile, line);
            phys->finalDriveAuto = std::stof(line);
            LOG(INFO) << "Vehicle final drive auto set to " << phys->finalDriveAuto;
            break;
        case (12):
            std::getline(*infile, line);
            phys->minRPM = std::stoi(line);
            LOG(INFO) << "Vehicle min RPM set to " << phys->minRPM << " RPM";
            break;
        case (13):
            std::getline(*infile, line);
            phys->maxRPM = std::stoi(line);
            LOG(INFO) << "Vehicle max RPM set to " << phys->maxRPM << " RPM";
            break;
        case (15):
            std::getline(*infile, line);
            phys->maxSpeed = std::stof(line);
            LOG(INFO) << "Vehicle max speed set to " << phys->maxSpeed << " m/s";
            break;
        default:
            // Do nothing and skip over
            if(!std::getline(*infile, line))
            {
                inFile = false;
            }
        }
    }
}