#include "Test/PhysXRackJointScene.h"
#include "TestRigidBodyCreate.h"
#include "../Render/RenderObjectAdapter.h"
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>
#include "Physics/PhysicsCommon.h"
#include <string>

PhysXRackJointScene::PhysXRackJointScene() 
    : TestSceneBase(TestSceneType::PHYSX_RACK_JOINT_SCENE)
{
}

PhysXRackJointScene::~PhysXRackJointScene()
{
}

void PhysXRackJointScene::Initialize()
{
    if (m_initialized)
        return;

    // Create physics scene with gravity
    PhysicsSceneCreateOptions sceneOptions;
    sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
    sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

    m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);

    // Create material for physics objects
    PhysicsMaterialCreateOptions materialOptions;
    materialOptions.m_StaticFriction = 0.5f;
    materialOptions.m_DynamicFriction = 0.5f;
    materialOptions.m_Restitution = 0.6f;
    materialOptions.m_Density = 10.0f;
    m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

    // Create the ground plane
    CreateGround();

    // Create the rack and pinion setup
    CreateRackAndPinionSetup();

    m_initialized = true;
}

void PhysXRackJointScene::Update(float deltaTime)
{
    if (!m_initialized || m_paused)
        return;

    m_elapsedTime += deltaTime;
    m_GlobalTime += deltaTime;
    
    // Update the hinge joint's drive velocity periodically for a more interesting demo
    if (m_HingeJoint)
    {
        // Apply a sinusoidal velocity to the hinge joint
        float velocityTarget = cosf(m_GlobalTime * 0.5f) * m_GearDriveVelocity;
        
    }
    
    // Update the physics simulation
    m_Scene->Tick(deltaTime);
}

void PhysXRackJointScene::Render()
{
    // No custom rendering needed
}

void PhysXRackJointScene::Cleanup()
{
    TestSceneBase::Cleanup();
}

void PhysXRackJointScene::Reset()
{
    Cleanup();
    Initialize();
}

void PhysXRackJointScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
    // No special mouse handling for this demo
}

void PhysXRackJointScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    if (key == ' ' && action == 1)
    {
        // Toggle drive direction
        m_DriveDirectionPositive = !m_DriveDirectionPositive;
        m_GearDriveVelocity = m_DriveDirectionPositive ? 3.0f : -3.0f;
    }
    else if (key == 'R' && action == 1)
    {
        // Reset the scene
        Reset();
    }
}

void PhysXRackJointScene::CreateGround()
{
    // Create a ground plane with normal pointing up (Y-axis)
    CollisionGeometryCreateOptions groundPlaneOptions;
    groundPlaneOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE;
    groundPlaneOptions.m_PlaneParams.m_Normal = MathLib::HVector3(0, 1, 0);
    groundPlaneOptions.m_PlaneParams.m_Distance = 0.0f;
    PhysicsPtr<IColliderGeometry> groundPlane = PhysicsEngineUtils::CreateColliderGeometry(groundPlaneOptions);

    PhysicsObjectCreateOptions groundPlaneObjectOptions;
    groundPlaneObjectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
    groundPlaneObjectOptions.m_Transform = MathLib::HTransform3::Identity();
    PhysicsPtr<IPhysicsObject> groundPlaneObject = PhysicsEngineUtils::CreateObject(groundPlaneObjectOptions);
    groundPlaneObject->AddColliderGeometry(groundPlane, MathLib::HTransform3::Identity());
    
    AddObject(groundPlaneObject, false);
}

PhysicsPtr<IPhysicsObject> PhysXRackJointScene::CreateGearWithBoxes(const MathLib::HTransform3& transform,
                                                                 const MathLib::HVector3& boxSize,
                                                                 int nbShapes)
{
    // Create a dynamic rigid body for the gear
    PhysicsObjectCreateOptions actorOptions;
    actorOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    actorOptions.m_Transform = transform;
    PhysicsPtr<IPhysicsObject> actor = PhysicsEngineUtils::CreateObject(actorOptions);
    
    // Add box shapes arranged in a circular pattern to create a "gear" like appearance
    for (int i = 0; i < nbShapes; i++)
    {
        // Calculate angle for this shape
        const float coeff = float(i) / float(nbShapes);
        const float angle = MathLib::H_PI * 0.5f * coeff;
        
        // Create box geometry
        CollisionGeometryCreateOptions boxOptions;
        boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
        boxOptions.m_BoxParams.m_HalfExtents = boxSize * 0.5f;
        PhysicsPtr<IColliderGeometry> boxGeometry = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
        
        // Calculate rotation matrix for this box
        MathLib::HMatrix3 rotMat;
        const float cos = cosf(angle);
        const float sin = sinf(angle);
        rotMat(0, 0) = rotMat(1, 1) = cos;
        rotMat(0, 1) = sin;
        rotMat(1, 0) = -sin;
        rotMat(2, 2) = 1.0f;
        
        // Create local transform for this box
        MathLib::HTransform3 localPose = MathLib::HTransform3::Identity();
        localPose.rotate(rotMat);
        
        // Attach box to the actor
        actor->AddColliderGeometry(boxGeometry, localPose);
    }
    
    // Update mass and inertia to account for all boxes
    IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(actor.get());
    if (rigidDynamic)
    {
        rigidDynamic->SetMass(1.0f);
    }
    
    return actor;
}

PhysicsPtr<IPhysicsObject> PhysXRackJointScene::CreateRackWithBoxes(const MathLib::HTransform3& transform,
                                                                  int nbTeeth,
                                                                  float rackLength)
{
    // Create a dynamic rigid body for the rack
    PhysicsObjectCreateOptions actorOptions;
    actorOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    actorOptions.m_Transform = transform;
    PhysicsPtr<IPhysicsObject> actor = PhysicsEngineUtils::CreateObject(actorOptions);
    
    // Add central box as the main body of the rack
    {
        CollisionGeometryCreateOptions boxOptions;
        boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
        boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(rackLength * 0.5f, 0.25f, 0.25f);
        PhysicsPtr<IColliderGeometry> boxGeometry = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
        actor->AddColliderGeometry(boxGeometry, MathLib::HTransform3::Identity());
    }
    
    // Create rotation matrix for teeth (45 degrees)
    MathLib::HMatrix3 rotMat;
    const float angle = MathLib::H_PI * 0.25f;
    const float cos = cosf(angle);
    const float sin = sinf(angle);
    rotMat(0, 0) = rotMat(1, 1) = cos;
    rotMat(0, 1) = sin;
    rotMat(1, 0) = -sin;
    rotMat(2, 2) = 1.0f;
    
    // Calculate spacing between teeth
    const float offset = rackLength / float(nbTeeth);
    
    // Add teeth boxes along the rack
    for (int i = 0; i < nbTeeth; i++)
    {
        CollisionGeometryCreateOptions boxOptions;
        boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
        boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(0.75f * 0.5f, 0.75f * 0.5f, 0.25f * 0.5f);
        PhysicsPtr<IColliderGeometry> boxGeometry = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
        
        // Position each tooth
        MathLib::HTransform3 localPose = MathLib::HTransform3::Identity();
        localPose.rotate(rotMat);
        localPose.translate(MathLib::HVector3((offset * i) - (rackLength * 0.5f) + (offset * 0.5f), 0.0f, 0.0f));
        
        actor->AddColliderGeometry(boxGeometry, localPose);
    }
    
    // Update mass and inertia
    IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(actor.get());
    if (rigidDynamic)
    {
        rigidDynamic->SetMass(1.0f);
    }
    
    return actor;
}

void PhysXRackJointScene::CreateRackAndPinionSetup()
{
    const float velocityTarget = 3.0f;
    const float radius = 3.0f;
    const float rackLength = 30.0f;

    const int nbPinionTeeth = int(radius) * 4;  // 'radius' teeth for PI/2
    const int nbRackTeeth = 15;

    const MathLib::HVector3 boxSize(radius, radius, 1.0f);
    const MathLib::HVector3 gearPos(0.0f, 10.0f, 0.0f);
    const MathLib::HVector3 rackPos(0.0f, 10.0f + radius + 1.5f, 0.0f);

    // Create gear actor
    m_Gear = CreateGearWithBoxes(
        MathLib::HTransform3(MathLib::HTranslation3(gearPos)),
        boxSize,
        nbPinionTeeth / 4);  // Using fewer shapes for the gear visual representation
    AddObject(m_Gear);

    // Create rack actor
    m_Rack = CreateRackWithBoxes(
        MathLib::HTransform3(MathLib::HTranslation3(rackPos)),
        nbRackTeeth,
        rackLength);
    AddObject(m_Rack);

    // Rotation to orient the revolute joint along the Z-axis
    MathLib::HMatrix3 x2z = MathLib::HMatrix3::Identity();
    x2z(0, 0) = 0.0f; x2z(0, 2) = 1.0f;
    x2z(2, 0) = 1.0f; x2z(2, 2) = 0.0f;

    // 使用正确的方式设置transform
    MathLib::HTransform3 localFrameA = MathLib::HTransform3::Identity();
    localFrameA.translate(gearPos);
    localFrameA.rotate(x2z);
    
    MathLib::HTransform3 localFrameB = MathLib::HTransform3::Identity();
    localFrameB.rotate(x2z);
    
    JointCreateOptions hingeOptions;
    hingeOptions.type = JointType::REVOLUTE;
    hingeOptions.objectA = nullptr; // Attach to the world
    hingeOptions.objectB = m_Gear;
    hingeOptions.localFrameA = localFrameA;
    hingeOptions.localFrameB = localFrameB;
    hingeOptions.collisionEnabled = false;
    
    m_HingeJoint = PhysicsEngineUtils::CreateJoint(hingeOptions);
    m_Scene->AddJoint(m_HingeJoint);

    MathLib::HTransform3 prismaticLocalFrameA = MathLib::HTransform3::Identity();
    prismaticLocalFrameA.translate(rackPos);
    
    MathLib::HTransform3 prismaticLocalFrameB = MathLib::HTransform3::Identity();
    
    JointCreateOptions prismaticOptions;
    prismaticOptions.type = JointType::PRISMATIC;
    prismaticOptions.objectA = nullptr; // Attach to the world
    prismaticOptions.objectB = m_Rack;
    prismaticOptions.localFrameA = prismaticLocalFrameA;
    prismaticOptions.localFrameB = prismaticLocalFrameB;
    prismaticOptions.collisionEnabled = false;
    
    m_PrismaticJoint = PhysicsEngineUtils::CreateJoint(prismaticOptions);
    m_Scene->AddJoint(m_PrismaticJoint);

    MathLib::HTransform3 rackLocalFrameA = MathLib::HTransform3::Identity();
    rackLocalFrameA.rotate(x2z);
    
    MathLib::HTransform3 rackLocalFrameB = MathLib::HTransform3::Identity();
    
    JointCreateOptions rackOptions;
    rackOptions.type = JointType::RACK_AND_PINION;
    rackOptions.objectA = m_Gear;
    rackOptions.objectB = m_Rack;
    rackOptions.localFrameA = rackLocalFrameA;
    rackOptions.localFrameB = rackLocalFrameB;
    rackOptions.collisionEnabled = false;
    
    m_RackJoint = PhysicsEngineUtils::CreateJoint(rackOptions);
    m_Scene->AddJoint(m_RackJoint);
    
} 