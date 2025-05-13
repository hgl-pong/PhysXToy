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
    // 如果已经初始化，不再重复初始化
    if (m_initialized)
        return;

    // 创建物理场景
    PhysicsSceneCreateOptions sceneOptions;
    sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
    sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

    m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);

    // 创建物理材质
    PhysicsMaterialCreateOptions materialOptions;
    materialOptions.m_StaticFriction = 0.5f;
    materialOptions.m_DynamicFriction = 0.5f;
    materialOptions.m_Restitution = 0.6f;
    materialOptions.m_Density = 10.0f;
    m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

    // 创建地面平面
    CreateGround();

    // 创建测试网格数据
    TestRigidBody::CreateTestingMeshData(); // Bunny
    auto physicsObjects = TestRigidBody::TestRigidBodyCreate();
    
    for (auto &physicsObject : physicsObjects)
    {
        m_Scene->AddPhysicsObject(physicsObject);
        AddPhysicsDebugRenderableObject(physicsObject);
        m_PhysicsObjects.push_back(physicsObject);
    }

    // 初始化完成
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
    // 渲染已通过RenderObjectAdapter处理
}

void TestScene::Cleanup()
{
    // 清理场景中的所有物理对象
    if (m_Scene)
    {
        for (auto &object : m_PhysicsObjects)
        {
            if (object)
            {
                m_Scene->RemovePhysicsObject(object);
            }
        }
    }
    
    // 清空对象列表
    m_PhysicsObjects.clear();
    
    // 释放物理场景和材质
    m_Scene.reset();
    m_Material.reset();
    
    // 标记为未初始化
    m_initialized = false;
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
    // 可以根据需要实现鼠标点击回调
}

void TestScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    if (key == ' ' && action == 1 && m_Renderer) // 空格键按下且渲染器可用
    {
        // 创建并发射一个球体
        CollisionGeometryCreateOptions options;
        options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
        options.m_SphereParams.m_Radius = 2.0f;

        PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
        
        // 获取相机信息并发射球体
        CreateDynamic(m_Renderer->GetActiveCamera()->GetTransform(), 
                     geometry, 
                     m_Renderer->GetActiveCamera()->GetDir() * 75);
    }
    else if ((key == 'B' || key == 'b') && action == 1) // B键按下
    {
        // 创建额外的刚体
        auto physicsObjects = TestRigidBody::TestRigidBodyCreate();
        for (auto &physicsObject : physicsObjects)
        {
            m_Scene->AddPhysicsObject(physicsObject);
            AddPhysicsDebugRenderableObject(physicsObject);
            m_PhysicsObjects.push_back(physicsObject);
        }
    }
}

void TestScene::CreateGround()
{
    // 创建地面平面
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
    
    if (m_Scene)
    {
        m_Scene->AddPhysicsObject(groundPlaneObject);
        m_PhysicsObjects.push_back(groundPlaneObject);
    }
}

PhysicsPtr<IPhysicsObject> TestScene::CreateDynamic(const MathLib::HTransform3 &t, 
                                                  PhysicsPtr<IColliderGeometry> &geometry, 
                                                  const MathLib::HVector3 &velocity)
{
    auto dynamic = TestRigidBody::CreateDynamic(t, geometry, velocity);
    if (m_Scene)
    {
        m_Scene->AddPhysicsObject(dynamic);
        AddPhysicsDebugRenderableObject(dynamic);
        m_PhysicsObjects.push_back(dynamic);
    }
    return dynamic;
}

void TestScene::AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject> &object)
{
    if (m_Renderer)
    {
        // 创建物理对象的可视化对象并添加到渲染器
        std::shared_ptr<RenderObject> renderable = std::make_shared<RenderObjectAdapter>(object);
        m_Renderer->AddRenderObject(renderable);
    }
} 