#include "Test/PhysXCustomConvexScene.h"
#include "TestRigidBodyCreate.h"
#include <random>
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>

PhysXCustomConvexScene::PhysXCustomConvexScene()
    : TestSceneBase(TestSceneType::PHYSX_CUSTOM_CONVEX_SCENE)
{
}

PhysXCustomConvexScene::~PhysXCustomConvexScene()
{
    Cleanup();
}

void PhysXCustomConvexScene::Initialize()
{
    PhysicsSceneCreateOptions sceneOptions;
    sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);
    sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
    m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);
    PhysicsEngineUtils::GetPhysicsEngine()->SetActiveScene(m_Scene);

    PhysicsMaterialCreateOptions materialOptions;
    materialOptions.m_StaticFriction = 0.5f;
    materialOptions.m_DynamicFriction = 0.5f;
    materialOptions.m_Restitution = 0.6f;
    m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

    CreateGround();

    float heights[] = { 1.0f, 1.25f, 1.5f, 1.75f };
    float radiuses[] = { 0.3f, 0.35f, 0.4f, 0.45f };
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> heightDist(0, 3);
    std::uniform_int_distribution<> radiusDist(0, 3);
    
    for (int i = 0; i < 20; ++i)
    {
        float height = heights[heightDist(gen)];
        float radius = radiuses[radiusDist(gen)];
        
        MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
        transform.translation() = MathLib::HVector3(-2.0f, 2.0f + i * 2.0f, 2.0f);
        
        MathLib::HQuaternion rotation;
        rotation = MathLib::HQuaternion(MathLib::HAngleAxis(MathLib::H_PI_2, MathLib::HVector3(0.0f, 0.0f, 1.0f)));
        transform.linear() = rotation.toRotationMatrix();
        
        m_physicsObjects.push_back(CreateCylinderActor(height, radius, transform));
    }
    
    for (int i = 0; i < 20; ++i)
    {
        float height = heights[heightDist(gen)];
        float radius = radiuses[radiusDist(gen)];
        
        MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
        transform.translation() = MathLib::HVector3(2.0f, 2.0f + i * 2.0f, -2.0f);
        
        MathLib::HQuaternion rotation;
        rotation = MathLib::HQuaternion(MathLib::HAngleAxis(MathLib::H_PI_2, MathLib::HVector3(0.0f, 0.0f, 1.0f)));
        transform.linear() = rotation.toRotationMatrix();
        
        m_physicsObjects.push_back(CreateConeActor(height, radius, transform));
    }
    
    m_initialized = true;
}

void PhysXCustomConvexScene::Update(float deltaTime)
{
    if (!m_paused)
    {
        m_elapsedTime += deltaTime;
        
        PerformRaycastTests();
        PerformSweepTests();
        PerformOverlapTests();
    }
}

void PhysXCustomConvexScene::Render()
{
}

void PhysXCustomConvexScene::Cleanup()
{
    m_physicsObjects.clear();
    
    TestSceneBase::Cleanup();
}

void PhysXCustomConvexScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
    if (button == 0 && action == 1)
    {
        IRenderer* renderer = GetRenderer();
        if (renderer && renderer->GetActiveCamera())
        {
            MathLib::HVector3 origin = renderer->GetActiveCamera()->GetEye();
            MathLib::HVector3 direction = renderer->GetActiveCamera()->GetDir();
            ShootSphere(origin, direction);
        }
    }
}

void PhysXCustomConvexScene::KeyBoardCallback(int key, int scancode, int action, int mods)
{
    if (key == ' ' && action == 1)
    {
        IRenderer* renderer = GetRenderer();
        if (renderer && renderer->GetActiveCamera())
        {
            MathLib::HVector3 origin = renderer->GetActiveCamera()->GetEye();
            MathLib::HVector3 direction = renderer->GetActiveCamera()->GetDir();
            ShootSphere(origin, direction);
        }
    }
    
    if (key == 'R' && action == 1)
    {
        Reset();
    }
    
    if (key == 'P' && action == 1)
    {
        m_paused = !m_paused;
    }
}

void PhysXCustomConvexScene::CreateGround()
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

PhysicsPtr<IPhysicsObject> PhysXCustomConvexScene::CreateCylinderActor(float height, float radius, 
                                                                      const MathLib::HTransform3& transform)
{
    CollisionGeometryCreateOptions options;
    options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
    options.m_CapsuleParams.m_Radius = radius;
    options.m_CapsuleParams.m_HalfHeight = height * 0.5f;
    PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
    
    PhysicsObjectCreateOptions objectOptions;
    objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    objectOptions.m_Transform = transform;
    PhysicsPtr<IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
    
    physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
    
    IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(physicsObject.get());
    if (rigidDynamic) {
        rigidDynamic->SetMass(10.0f);
    }
    
    AddObject(physicsObject);
    
    return physicsObject;
}

PhysicsPtr<IPhysicsObject> PhysXCustomConvexScene::CreateConeActor(float height, float radius, 
                                                                  const MathLib::HTransform3& transform)
{
    CollisionGeometryCreateOptions options;
    options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
    options.m_CapsuleParams.m_Radius = radius;
    options.m_CapsuleParams.m_HalfHeight = height * 0.5f;
    PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
    
    PhysicsObjectCreateOptions objectOptions;
    objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    objectOptions.m_Transform = transform;
    PhysicsPtr<IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
    
    physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
    
    IRigidDynamic* rigidDynamic = dynamic_cast<IRigidDynamic*>(physicsObject.get());
    if (rigidDynamic) {
        rigidDynamic->SetMass(10.0f);
    }
    
    AddObject(physicsObject);
    
    return physicsObject;
}

void PhysXCustomConvexScene::ShootSphere(const MathLib::HVector3& origin, const MathLib::HVector3& direction)
{
    CollisionGeometryCreateOptions options;
    options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
    options.m_SphereParams.m_Radius = 0.5f;
    PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
    
    MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
    transform.translation() = origin;
    
    auto dynamic = TestRigidBody::CreateDynamic(transform, geometry, direction * 30.0f);
    
    AddObject(dynamic);
    m_physicsObjects.push_back(dynamic);
}

void PhysXCustomConvexScene::PerformRaycastTests()
{

}

void PhysXCustomConvexScene::PerformSweepTests()
{

}

void PhysXCustomConvexScene::PerformOverlapTests()
{

} 