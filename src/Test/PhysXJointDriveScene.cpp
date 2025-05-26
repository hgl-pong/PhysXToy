#include "Test/PhysXJointDriveScene.h"
#include "TestRigidBodyCreate.h"
#include "../Render/RenderObjectAdapter.h"
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>
#include <string>

PhysXJointDriveScene::PhysXJointDriveScene() 
    : TestSceneBase(TestSceneType::PHYSX_JOINT_DRIVE_SCENE)
{
}

PhysXJointDriveScene::~PhysXJointDriveScene()
{
}

void PhysXJointDriveScene::Initialize()
{
    if (m_initialized)
        return;

    // Create physics scene with no gravity to isolate drive effects
    PhysicsSceneCreateOptions sceneOptions;
    sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
    sceneOptions.m_Gravity = MathLib::HVector3(0.0f, 0.0f, 0.0f); // Disable gravity

    m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);

    PhysicsMaterialCreateOptions materialOptions;
    materialOptions.m_StaticFriction = 0.5f;
    materialOptions.m_DynamicFriction = 0.5f;
    materialOptions.m_Restitution = 0.6f;
    materialOptions.m_Density = 1.0f;
    m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

    CreateGround();
    CreateScene();

    m_initialized = true;
}

void PhysXJointDriveScene::Update(float deltaTime)
{
    if (!m_initialized || m_paused)
        return;

    m_elapsedTime += deltaTime;

    if (m_Scene)
    {
        m_Scene->Tick(deltaTime);
    }
}

void PhysXJointDriveScene::Render()
{
}

void PhysXJointDriveScene::Reset()
{
    Cleanup();
    Initialize();
}

void PhysXJointDriveScene::Cleanup()
{
    if (m_d6Joint && m_Scene)
    {
        m_Scene->RemoveJoint(m_d6Joint);
    }
    m_d6Joint.reset();
    m_objectA.reset();
    m_objectB.reset();
    
    TestSceneBase::Cleanup();
}

void PhysXJointDriveScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
}

void PhysXJointDriveScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    auto renderer = GetRenderer();
    if (key == ' ' && action == 1 && renderer)
    {
        // Fire a sphere when spacebar is pressed
        CollisionGeometryCreateOptions options;
        options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
        options.m_SphereParams.m_Radius = 1.0f;

        PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
        
        auto dynamic = TestRigidBody::CreateDynamic(renderer->GetActiveCamera()->GetTransform(),
                     geometry, 
                     renderer->GetActiveCamera()->GetDir() * 50);
        AddObject(dynamic);
    }
    
    // Handle F1-F4 keys (scancode 1-4 in some systems)
    if (action == 1)
    {
        bool recreateScene = false;
        
        switch (key)
        {
        case '1': // F1 equivalent - Toggle joint frame A rotation
            m_changeJointFrameARotation = !m_changeJointFrameARotation;
            recreateScene = true;
            break;
        case '2': // F2 equivalent - Toggle object A type
            m_changeObjectAType = !m_changeObjectAType;
            recreateScene = true;
            break;
        case '3': // F3 equivalent - Toggle joint frame B rotation
            m_changeJointFrameBRotation = !m_changeJointFrameBRotation;
            recreateScene = true;
            break;
        case '4': // F4 equivalent - Toggle object B rotation
            m_changeObjectBRotation = !m_changeObjectBRotation;
            recreateScene = true;
            break;
        case '5': // F5 equivalent - Switch drive type
            m_sceneIndex = (m_sceneIndex + 1) % MAX_SCENE_INDEX;
            recreateScene = true;
            break;
        }
        
        if (recreateScene)
        {
            CreateScene();
        }
    }
}

void PhysXJointDriveScene::CreateGround()
{
    // Create a ground plane
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

void PhysXJointDriveScene::CreateScene()
{
    // Clean up existing objects
    if (m_d6Joint && m_Scene)
    {
        m_Scene->RemoveJoint(m_d6Joint);
    }
    if (m_objectA)
    {
        RemoveObject(m_objectA);
    }
    if (m_objectB)
    {
        RemoveObject(m_objectB);
    }
    
    m_d6Joint.reset();
    m_objectA.reset();
    m_objectB.reset();

    // Create box geometry for both objects
    const MathLib::HVector3 boxExtents(0.5f, 0.5f, 0.5f);
    CollisionGeometryCreateOptions boxOptions;
    boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
    boxOptions.m_BoxParams.m_HalfExtents = boxExtents;
    PhysicsPtr<IColliderGeometry> boxGeometry = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);

    // Calculate rotation for 45 degree tilt
    const MathLib::HReal angle = -MathLib::H_PI / 4.0f;
    MathLib::HQuaternion rotZ(MathLib::HAngleAxis(angle, MathLib::HVector3(0, 0, 1)));

    // Create object A (first box)
    MathLib::HTransform3 transformA = MathLib::HTransform3::Identity();
    transformA.translate(MathLib::HVector3(0.0f, 2.0f, -20.0f));
    
    PhysicsObjectCreateOptions objectAOptions;
    if (m_changeObjectAType)
    {
        objectAOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
        objectAOptions.m_IsKinematic = true;
    }
    else
    {
        objectAOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
    }
    objectAOptions.m_Transform = transformA;
    objectAOptions.m_MaterialOptions.m_Density = 1.0f;
    
    m_objectA = PhysicsEngineUtils::CreateObject(objectAOptions);
    m_objectA->AddColliderGeometry(boxGeometry, MathLib::HTransform3::Identity());
    AddObject(m_objectA);

    // Create object B (second box)
    MathLib::HTransform3 transformB = transformA;
    transformB.translate(MathLib::HVector3(boxExtents.x() * 2.0f, 0.0f, 0.0f));
    if (m_changeObjectBRotation)
    {
        transformB.rotate(rotZ);
    }
    
    PhysicsObjectCreateOptions objectBOptions;
    objectBOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    objectBOptions.m_Transform = transformB;
    objectBOptions.m_MaterialOptions.m_Density = 1.0f;
    
    m_objectB = PhysicsEngineUtils::CreateObject(objectBOptions);
    m_objectB->AddColliderGeometry(boxGeometry, MathLib::HTransform3::Identity());
    AddObject(m_objectB);

    // Create D6 joint with drive
    CreateD6JointWithDrive(m_sceneIndex);
}

void PhysXJointDriveScene::CreateD6JointWithDrive(uint32_t sceneIndex)
{
    if (!m_objectA || !m_objectB)
        return;

    // Calculate joint frames
    MathLib::HQuaternion rotZ(MathLib::HAngleAxis(-MathLib::H_PI / 4.0f, MathLib::HVector3(0, 0, 1)));
    
    MathLib::HTransform3 jointFrameA = MathLib::HTransform3::Identity();
    if (m_changeJointFrameARotation)
        jointFrameA.rotate(rotZ);
    
    MathLib::HTransform3 jointFrameB = MathLib::HTransform3::Identity();
    if (m_changeJointFrameBRotation)
        jointFrameB.rotate(rotZ);

    // Create D6 joint
    JointCreateOptions options;
    options.type = JointType::D6;
    options.objectA = m_objectA;
    options.objectB = m_objectB;
    options.localFrameA = jointFrameA;
    options.localFrameB = jointFrameB;
    options.collisionEnabled = false;
    
    m_d6Joint = PhysicsEngineUtils::CreateJoint(options);
    
    // Set all DOFs to free to make sure none interferes with the drive
    JointLimitOptions limitOptions;
    limitOptions.m_XAxis.m_IsLimited = false;
    limitOptions.m_YAxis.m_IsLimited = false;
    limitOptions.m_ZAxis.m_IsLimited = false;
    limitOptions.m_Swing1.m_IsLimited = false;
    limitOptions.m_Swing2.m_IsLimited = false;
    limitOptions.m_Twist.m_IsLimited = false;
    
    m_d6Joint->SetJointLimits(limitOptions);

    // Configure drive based on scene index
    JointDriveSettings driveSettings;
    driveSettings.m_Enabled = true;
    driveSettings.m_Stiffness = 0.0f;
    driveSettings.m_Damping = 1000.0f;
    driveSettings.m_ForceLimit = FLT_MAX;
    driveSettings.m_IsAcceleration = true;
    driveSettings.m_DriveType = JointDriveType::VELOCITY;
    driveSettings.m_TargetVelocity = 1.0f;

    switch (sceneIndex)
    {
    case 0: // Linear drive along X
        m_d6Joint->SetDrive(JointAxis::X, driveSettings);
        break;
    case 1: // Angular twist drive around X
        m_d6Joint->SetDrive(JointAxis::TWIST, driveSettings);
        break;
    case 2: // Angular swing drive around Y
        m_d6Joint->SetDrive(JointAxis::SWING1, driveSettings);
        break;
    case 3: // Angular SLERP drive around Y
        m_d6Joint->SetDrive(JointAxis::SLERP, driveSettings);
        break;
    }

    if (m_Scene)
    {
        m_Scene->AddJoint(m_d6Joint);
    }
} 