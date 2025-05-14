#include "Test/TestScene.h"
#include "TestRigidBodyCreate.h"
#include "../Render/RenderObjectAdapter.h"
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>

TestScene::TestScene() 
    : TestSceneBase("Default physics scene with ground plane and physics objects")
{
}

TestScene::~TestScene()
{
    Cleanup();
}

void TestScene::Initialize()
{
    if (m_initialized)
        return;

    PhysicsSceneCreateOptions sceneOptions;
    sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
    sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

    m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);

    PhysicsMaterialCreateOptions materialOptions;
    materialOptions.m_StaticFriction = 0.5f;
    materialOptions.m_DynamicFriction = 0.5f;
    materialOptions.m_Restitution = 0.6f;
    materialOptions.m_Density = 10.0f;
    m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

    CreateGround();

    TestRigidBody::CreateTestingMeshData(); // Bunny
    auto physicsObjects = TestRigidBody::TestRigidBodyCreate();
    
    for (auto &physicsObject : physicsObjects)
    {
        AddObject(physicsObject);
    }

    m_initialized = true;
}

void TestScene::Update(float deltaTime)
{
    if (!m_initialized || m_paused)
        return;

    m_elapsedTime += deltaTime;

    if (m_Scene)
    {
        m_Scene->Tick(deltaTime);
    }
}

void TestScene::Render()
{
}

std::string TestScene::GetName() const
{
    return "Default Physics Scene";
}

void TestScene::Reset()
{
    Cleanup();
    Initialize();
}

void TestScene::Pause()
{
    m_paused = true;
}

void TestScene::Resume()
{
    m_paused = false;
}

void TestScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
}

void TestScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    if (key == ' ' && action == 1 && m_Renderer)
    {
        CollisionGeometryCreateOptions options;
        options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
        options.m_SphereParams.m_Radius = 2.0f;

        PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
        
        auto dynamic = TestRigidBody::CreateDynamic(m_Renderer->GetActiveCamera()->GetTransform(),
                     geometry, 
                     m_Renderer->GetActiveCamera()->GetDir() * 75);
        AddObject(dynamic);
    }
    else if ((key == 'B' || key == 'b') && action == 1)
    {
        auto physicsObjects = TestRigidBody::TestRigidBodyCreate();
        for (auto &physicsObject : physicsObjects)
        {
            AddObject(physicsObject);
        }
    }
}

void TestScene::CreateGround()
{
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
