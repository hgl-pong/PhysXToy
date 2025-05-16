#include "Test/PhysXSplitSimScene.h"
#include "TestRigidBodyCreate.h"
#include "../Render/RenderObjectAdapter.h"
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>
#include <string>

PhysXSplitSimScene::PhysXSplitSimScene() 
    : TestSceneBase(TestSceneType::PHYSX_SPLIT_SIM_SCENE)
{
}

PhysXSplitSimScene::~PhysXSplitSimScene()
{
}

void PhysXSplitSimScene::Initialize()
{
    if (m_initialized)
        return;

    // Create physics scene with default settings
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

    // Create ground plane
    CreateGround();
    
    // Create kinematic platforms
    CreateKinematics();
    
    // Create dynamic objects that will interact with the kinematics
    CreateDynamics();

    m_initialized = true;
    m_isFirstFrame = true;
    m_time = 0.0f;
}

void PhysXSplitSimScene::Update(float deltaTime)
{
    if (!m_initialized || m_paused)
        return;

    m_elapsedTime += deltaTime;
    const float timeStep = 1.0f/60.0f;

    if (m_useNoLagMode)
    {
        // No lag mode (physics time matches application time)
        if (m_isFirstFrame)
        {
            // Run the first frame's collision detection
            m_Scene->Tick(timeStep);
            m_isFirstFrame = false;
        }
        
        // Update the kinematic target poses in parallel with collision running
        UpdateKinematicTargets(timeStep);
        
        // Apply the computed and buffered kinematic target poses
        ApplyKinematicTargets();
        
        // Perform dynamics update
        m_Scene->Tick(timeStep);
    }
    else
    {
        // One frame lag mode (physics time is one step behind application time)
        UpdateKinematicTargets(timeStep);
        
        if (!m_isFirstFrame)
        {
            // Apply the computed and buffered kinematic target poses
            ApplyKinematicTargets();
            
            // Perform dynamics update
            m_Scene->Tick(timeStep);
        }
        else
        {
            // For the first frame, just apply kinematic targets
            ApplyKinematicTargets();
            m_isFirstFrame = false;
        }
    }
}

void PhysXSplitSimScene::Render()
{
    // No custom rendering needed as objects are rendered through the renderer system
}

void PhysXSplitSimScene::Cleanup()
{
    TestSceneBase::Cleanup();
}

void PhysXSplitSimScene::Reset()
{
    Cleanup();
    Initialize();
}

void PhysXSplitSimScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
    // No special mouse handling needed for this demo
}

void PhysXSplitSimScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    if (key == ' ' && action == 1)
    {
        // Fire a sphere when spacebar is pressed
        auto renderer = GetRenderer();
        if (renderer)
        {
            CollisionGeometryCreateOptions options;
            options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            options.m_SphereParams.m_Radius = 0.5f;

            PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
            
            auto dynamic = TestRigidBody::CreateDynamic(renderer->GetActiveCamera()->GetTransform(),
                         geometry, 
                         renderer->GetActiveCamera()->GetDir() * 50);
            AddObject(dynamic);
        }
    }
    else if (key == 'M' && action == 1)
    {
        // Toggle between no lag mode and one frame lag mode
        m_useNoLagMode = !m_useNoLagMode;
        m_isFirstFrame = true;
    }
}

void PhysXSplitSimScene::CreateGround()
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

void PhysXSplitSimScene::CreateKinematics()
{
    // Create a grid of kinematic platforms
    const float ScaleX = KINE_SCALE;
    const float ScaleY = KINE_SCALE;
    const float YScale = 0.4f;
    
    // Create box shape for all kinematic platforms
    CollisionGeometryCreateOptions geomOptions;
    geomOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
    geomOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(1.5f, 0.2f, 1.5f);
    PhysicsPtr<IColliderGeometry> boxGeometry = PhysicsEngineUtils::CreateColliderGeometry(geomOptions);
    
    for (int y = 0; y < NB_KINE_Y; y++)
    {
        for (int x = 0; x < NB_KINE_X; x++)
        {
            const float xf = (float(x) - float(NB_KINE_X) * 0.5f) * ScaleX;
            const float zf = (float(y) - float(NB_KINE_Y) * 0.5f) * ScaleY;
            
            MathLib::HTransform3 pose = MathLib::HTransform3::Identity();
            pose.translate(MathLib::HVector3(xf, 0.2f + YScale, zf));
            
            PhysicsObjectCreateOptions objectOptions;
            objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
            objectOptions.m_Transform = pose;
            
            PhysicsPtr<IPhysicsObject> body = PhysicsEngineUtils::CreateObject(objectOptions);
            body->AddColliderGeometry(boxGeometry, MathLib::HTransform3::Identity());
            
            // Set as kinematic
            IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(body.get());
            if (rigidDynamic)
            {
                rigidDynamic->SetKinematic(true);
            }
            
            AddObject(body);
            m_Kinematics[y][x] = body;
        }
    }
}

void PhysXSplitSimScene::CreateDynamics()
{
    // Create various dynamic objects that will interact with the kinematic platforms
    const int NbX = 8;
    const int NbY = 8;
    const int NbLayers = 3;
    const float YScale = 0.4f;
    const float YStart = 6.0f;
    
    // Create different geometry types for variety
    CollisionGeometryCreateOptions boxOptions;
    boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
    boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(0.2f, 0.1f, 0.2f);
    PhysicsPtr<IColliderGeometry> boxGeometry = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
    
    CollisionGeometryCreateOptions sphereOptions;
    sphereOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
    sphereOptions.m_SphereParams.m_Radius = 0.2f;
    PhysicsPtr<IColliderGeometry> sphereGeometry = PhysicsEngineUtils::CreateColliderGeometry(sphereOptions);
    
    CollisionGeometryCreateOptions capsuleOptions;
    capsuleOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
    capsuleOptions.m_CapsuleParams.m_Radius = 0.2f;
    capsuleOptions.m_CapsuleParams.m_HalfHeight = 0.5f;
    PhysicsPtr<IColliderGeometry> capsuleGeometry = PhysicsEngineUtils::CreateColliderGeometry(capsuleOptions);
    
    for (int j = 0; j < NbLayers; j++)
    {
        const float angle = float(j) * 0.08f;
        const MathLib::HTransform3 rot = MathLib::HTransform3(MathLib::Rotation(MathLib::HVector3(0.0f, 1.0f, 0.0f), angle));
        
        const float ScaleX = 4.0f;
        const float ScaleY = 4.0f;
        
        for (int y = 0; y < NbY; y++)
        {
            for (int x = 0; x < NbX; x++)
            {
                const float xf = (float(x) - float(NbX) * 0.5f) * ScaleX;
                const float zf = (float(y) - float(NbY) * 0.5f) * ScaleY;
                
                PhysicsPtr<IPhysicsObject> dynamic;
                MathLib::HVector3 pos(xf, YStart + float(j) * YScale, zf);
                
                // Create different object types for each layer
                int v = j % 3;
                switch (v)
                {
                    case 0:
                    {
                        // Create box
                        MathLib::HTransform3 pose = rot;
                        pose.translate(pos);
                        dynamic = TestRigidBody::CreateDynamic(pose, boxGeometry);
                        break;
                    }
                    case 1:
                    {
                        // Create sphere
                        MathLib::HTransform3 pose = MathLib::HTransform3::Identity();
                        pose.translate(pos);
                        dynamic = TestRigidBody::CreateDynamic(pose, sphereGeometry);
                        break;
                    }
                    default:
                    {
                        // Create capsule
                        MathLib::HTransform3 pose = rot;
                        pose.translate(pos);
                        dynamic = TestRigidBody::CreateDynamic(pose, capsuleGeometry);
                        break;
                    }
                }
                
                AddObject(dynamic);
            }
        }
    }
}

void PhysXSplitSimScene::UpdateKinematicTargets(float deltaTime)
{
    m_time += deltaTime;
    
    const float YScale = 0.4f;
    const float Coeff = 0.2f;
    const float ScaleX = KINE_SCALE;
    const float ScaleY = KINE_SCALE;
    
    for (int y = 0; y < NB_KINE_Y; y++)
    {
        for (int x = 0; x < NB_KINE_X; x++)
        {
            const float xf = (float(x) - float(NB_KINE_X) * 0.5f) * ScaleX;
            const float zf = (float(y) - float(NB_KINE_Y) * 0.5f) * ScaleY;
            
            // Create wave-like motion using sin function
            const float h = sin(m_time * 2.0f + float(x) * Coeff + float(y) * Coeff) * 2.0f;
            
            MathLib::HTransform3 motion = MathLib::HTransform3::Identity();
            motion.translate(MathLib::HVector3(xf, h + 2.0f + YScale, zf));
            
            m_KinematicTargets[y][x] = motion;
        }
    }
}

void PhysXSplitSimScene::ApplyKinematicTargets()
{
    for (int y = 0; y < NB_KINE_Y; y++)
    {
        for (int x = 0; x < NB_KINE_X; x++)
        {
            PhysicsPtr<IPhysicsObject> kine = m_Kinematics[y][x];
            if (kine)
            {
                IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(kine.get());
                if (rigidDynamic)
                {
                    rigidDynamic->SetTransform(m_KinematicTargets[y][x]);
                }
            }
        }
    }
} 