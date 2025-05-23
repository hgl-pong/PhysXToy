#include "Test/PhysXGearJointScene.h"
#include "TestRigidBodyCreate.h"
#include "../Render/RenderObjectAdapter.h"
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>
#include "Physics/PhysicsCommon.h"
#include <string>

PhysXGearJointScene::PhysXGearJointScene() 
    : TestSceneBase(TestSceneType::PHYSX_GEAR_JOINT_SCENE)
{
}

PhysXGearJointScene::~PhysXGearJointScene()
{
}

void PhysXGearJointScene::Initialize()
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

    // Create the gear joint setup
    CreateGearJointSetup();

    m_initialized = true;
}

void PhysXGearJointScene::Update(float deltaTime)
{
    if (!m_initialized || m_paused)
        return;

    m_elapsedTime += deltaTime;
    m_GlobalTime += deltaTime;
    
    // Update the drive velocity based on automatic control or user input
    if (m_AutoVelocityControl)
    {
        // Apply a sinusoidal velocity pattern for more interesting motion
        float velocityTarget = sinf(m_GlobalTime) * 3.0f;
        
        // PhysicsLib doesn't provide a direct interface to set drive velocity on joints
        // In a real implementation, we would need to:
        // 1. Add SetDriveVelocity method to the IPhysicsJoint interface
        // 2. Implement it in PhysicsJoint class to call m_PxRevoluteJoint->setDriveVelocity()
        // 3. Call that method here
        
        // For now, we just apply an equivalent force to create the motion
        if (m_Gear0 && m_HingeJoint0)
        {
            IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(m_Gear0.get());
            if (rigidDynamic)
            {
                // Apply torque around the Z axis to drive the gear
                rigidDynamic->AddTorque(MathLib::HVector3(0.0f, 0.0f, velocityTarget * 5.0f));
            }
        }
    }
    else if (m_DriveVelocity != 0.0f)
    {
        // Apply fixed drive velocity when not in automatic mode
        IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(m_Gear0.get());
        if (rigidDynamic)
        {
            // Apply torque around the Z axis to drive the gear
            rigidDynamic->AddTorque(MathLib::HVector3(0.0f, 0.0f, m_DriveVelocity * 5.0f));
        }
    }
    
    // Update the physics simulation
    m_Scene->Tick(deltaTime);
}

void PhysXGearJointScene::Render()
{
    // No custom rendering needed
}

void PhysXGearJointScene::Cleanup()
{
    TestSceneBase::Cleanup();
}

void PhysXGearJointScene::Reset()
{
    Cleanup();
    Initialize();
}

void PhysXGearJointScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
    // No special mouse handling for this demo
}

void PhysXGearJointScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    if (key == ' ' && action == 1)
    {
        // Toggle automatic velocity control
        m_AutoVelocityControl = !m_AutoVelocityControl;
    }
    else if (key == 'R' && action == 1)
    {
        // Reset the scene
        Reset();
    }
    else if (key == 'A' && action == 1)
    {
        // Increase drive velocity
        m_DriveVelocity += 0.5f;
    }
    else if (key == 'Z' && action == 1)
    {
        // Decrease drive velocity
        m_DriveVelocity -= 0.5f;
    }
}

void PhysXGearJointScene::CreateGround()
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

PhysicsPtr<IPhysicsObject> PhysXGearJointScene::CreateGearWithBoxes(const MathLib::HTransform3& transform,
                                                                 const HBoxGeometry& boxGeom,
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
        boxOptions.m_BoxParams.m_HalfExtents = boxGeom.extents;
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

void PhysXGearJointScene::CreateGearJointSetup()
{
    const float velocityTarget = m_DriveVelocity;
    const float radius0 = 5.0f;
    const float radius1 = 2.0f;

    // Calculate extended geometry for gear teeth
    const float extent0 = radius0 * sqrtf(2.0f);
    const float extent1 = radius1 * sqrtf(2.0f);
    const float teethLength0 = extent0 - radius0;
    const float teethLength1 = extent1 - radius1;
    const float extra = (teethLength0 + teethLength1) * 0.75f;

    // Create box geometries for the gears
    HBoxGeometry boxGeom0;
    boxGeom0.extents = MathLib::HVector3(radius0, radius0, 0.5f);

    HBoxGeometry boxGeom1;
    boxGeom1.extents = MathLib::HVector3(radius1, radius1, 0.25f);

    // Position the gears
    const MathLib::HVector3 boxPos0(0.0f, 10.0f, 0.0f);
    const MathLib::HVector3 boxPos1(radius0 + radius1 + extra, 10.0f, 0.0f);

    // Create the first gear actor
    MathLib::HTransform3 transform0 = MathLib::HTransform3::Identity();
    transform0.translate(boxPos0);
    m_Gear0 = CreateGearWithBoxes(transform0, boxGeom0, int(radius0));
    AddObject(m_Gear0);

    // Create the second gear actor
    MathLib::HTransform3 transform1 = MathLib::HTransform3::Identity();
    transform1.translate(boxPos1);
    m_Gear1 = CreateGearWithBoxes(transform1, boxGeom1, int(radius1));
    AddObject(m_Gear1);

    // Rotation to orient the revolute joints along the Z-axis
    MathLib::HMatrix3 x2z = MathLib::HMatrix3::Identity();
    x2z(0, 0) = 0.0f; x2z(0, 2) = 1.0f;
    x2z(2, 0) = 1.0f; x2z(2, 2) = 0.0f;

    // Create the first hinge joint attached to the world
    MathLib::HTransform3 localFrameA0 = MathLib::HTransform3::Identity();
    localFrameA0.translate(boxPos0);
    localFrameA0.rotate(x2z);
    
    MathLib::HTransform3 localFrameB0 = MathLib::HTransform3::Identity();
    localFrameB0.rotate(x2z);
    
    JointCreateOptions hingeOptions0;
    hingeOptions0.type = JointType::REVOLUTE;
    hingeOptions0.objectA = nullptr; // Attach to the world
    hingeOptions0.objectB = m_Gear0;
    hingeOptions0.localFrameA = localFrameA0;
    hingeOptions0.localFrameB = localFrameB0;
    hingeOptions0.collisionEnabled = false;
    
    m_HingeJoint0 = PhysicsEngineUtils::CreateJoint(hingeOptions0);
    m_Scene->AddJoint(m_HingeJoint0);

    // Create the second hinge joint attached to the world
    MathLib::HTransform3 localFrameA1 = MathLib::HTransform3::Identity();
    localFrameA1.translate(boxPos1);
    localFrameA1.rotate(x2z);
    
    MathLib::HTransform3 localFrameB1 = MathLib::HTransform3::Identity();
    localFrameB1.rotate(x2z);
    
    JointCreateOptions hingeOptions1;
    hingeOptions1.type = JointType::REVOLUTE;
    hingeOptions1.objectA = nullptr; // Attach to the world
    hingeOptions1.objectB = m_Gear1;
    hingeOptions1.localFrameA = localFrameA1;
    hingeOptions1.localFrameB = localFrameB1;
    hingeOptions1.collisionEnabled = false;
    
    m_HingeJoint1 = PhysicsEngineUtils::CreateJoint(hingeOptions1);
    m_Scene->AddJoint(m_HingeJoint1);

    // Create the gear joint connecting the two gears
    MathLib::HTransform3 gearLocalFrameA = MathLib::HTransform3::Identity();
    gearLocalFrameA.rotate(x2z);
    
    MathLib::HTransform3 gearLocalFrameB = MathLib::HTransform3::Identity();
    gearLocalFrameB.rotate(x2z);
    
    JointCreateOptions gearOptions;
    gearOptions.type = JointType::GEAR;
    gearOptions.objectA = m_Gear0;
    gearOptions.objectB = m_Gear1;
    gearOptions.localFrameA = gearLocalFrameA;
    gearOptions.localFrameB = gearLocalFrameB;
    gearOptions.collisionEnabled = false;
    
    m_GearJoint = PhysicsEngineUtils::CreateJoint(gearOptions);
    m_Scene->AddJoint(m_GearJoint);
    
    // Set initial drive velocity on the first hinge joint
    // Note: This would need to be implemented through PhysicsLib interfaces
} 