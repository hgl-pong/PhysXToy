#include "Test/PhysXGyroscopicScene.h"
#include "TestRigidBodyCreate.h"
#include "../Render/RenderObjectAdapter.h"
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>
#include <string>

PhysXGyroscopicScene::PhysXGyroscopicScene() 
    : TestSceneBase(TestSceneType::PHYSX_GYROSCOPIC_SCENE)
{
}

PhysXGyroscopicScene::~PhysXGyroscopicScene()
{
}

void PhysXGyroscopicScene::Initialize()
{
    if (m_initialized)
        return;

    // Create physics scene with zero gravity to better show gyroscopic effect
    PhysicsSceneCreateOptions sceneOptions;
    sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
    sceneOptions.m_Gravity = MathLib::HVector3(0.0f, 0.0f, 0.0f); // Zero gravity to better observe gyroscopic effect

    m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);

    // Create material for physics objects
    PhysicsMaterialCreateOptions materialOptions;
    materialOptions.m_StaticFriction = 0.5f;
    materialOptions.m_DynamicFriction = 0.5f;
    materialOptions.m_Restitution = 0.6f;
    materialOptions.m_Density = 10.0f;
    m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

    // Create the tumbling object
    CreateTumblingObject();

    m_initialized = true;
}

void PhysXGyroscopicScene::Update(float deltaTime)
{
    if (!m_initialized || m_paused)
        return;

    m_elapsedTime += deltaTime;
    
    // Update the physics simulation
    m_Scene->Tick(1.0f/60.0f);
}

void PhysXGyroscopicScene::Render()
{
    // No custom rendering needed
}

void PhysXGyroscopicScene::Cleanup()
{
    TestSceneBase::Cleanup();
}

void PhysXGyroscopicScene::Reset()
{
    Cleanup();
    Initialize();
}

void PhysXGyroscopicScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
    // No special mouse handling for this demo
}

void PhysXGyroscopicScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    if (key == ' ' && action == 1)
    {
        // Toggle gyroscopic forces on/off
        m_gyroscopicForcesEnabled = !m_gyroscopicForcesEnabled;
        
        if (m_tumblingObject)
        {
            IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(m_tumblingObject.get());
            if (rigidDynamic)
            {
                rigidDynamic->EnableGyroscopicForces(m_gyroscopicForcesEnabled);
            }
        }
    }
    else if (key == 'R' && action == 1)
    {
        // Reset the scene
        Reset();
    }
}

void PhysXGyroscopicScene::CreateTumblingObject()
{
    // Create the tumbling object as an asymmetric body for demonstrating the Dzhanibekov effect
    
    // Central box shape (the "body" of the object)
    CollisionGeometryCreateOptions boxOptions;
    boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
    boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(0.05f, 0.5f, 0.05f);
    PhysicsPtr<IColliderGeometry> centralGeometry = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);

    // Create the main object
    PhysicsObjectCreateOptions objectOptions;
    objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    objectOptions.m_Transform = MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0.0f, 1.0f, 0.0f)));
    
    m_tumblingObject = PhysicsEngineUtils::CreateObject(objectOptions);
    m_tumblingObject->AddColliderGeometry(centralGeometry, MathLib::HTransform3::Identity());

    // Add side wing to create asymmetry
    CollisionGeometryCreateOptions wingOptions;
    wingOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
    wingOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(0.1f, 0.05f, 0.05f);
    PhysicsPtr<IColliderGeometry> wingGeometry = PhysicsEngineUtils::CreateColliderGeometry(wingOptions);
    
    // Add the wing with an offset to create asymmetry
    MathLib::HTransform3 wingTransform = MathLib::HTransform3::Identity();
    wingTransform.translate(MathLib::HVector3(0.1f, 0.0f, 0.0f));
    m_tumblingObject->AddColliderGeometry(wingGeometry, wingTransform);
    
    // Update the mass and inertia properties
    IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(m_tumblingObject.get());
    if (rigidDynamic)
    {
        // Set mass properties
        rigidDynamic->SetMass(1.0f);
        
        // Enable gyroscopic forces
        rigidDynamic->EnableGyroscopicForces(m_gyroscopicForcesEnabled);
        
        // Set initial angular velocity to start the tumbling motion
        rigidDynamic->SetAngularVelocity(MathLib::HVector3(7.5f, 5.0f, 0.0f));
        
        // Set zero angular damping to maintain the motion
        rigidDynamic->SetAngularDamping(0.0f);
    }
    
    // Add to scene
    AddObject(m_tumblingObject);
} 