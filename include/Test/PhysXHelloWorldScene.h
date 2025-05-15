#pragma once

#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"
#include <memory>
#include <vector>
#include "TestRigidBodyCreate.h"
#include <Math/GraphicUtils/Camara.h>
class PhysXHelloWorldScene : public TestSceneBase
{
public:
    PhysXHelloWorldScene() : 
        TestSceneBase(TestSceneType::PHYSX_HELLO_WORLD)
    {
    }

    ~PhysXHelloWorldScene() override
    {
    }

    void Initialize() override
    {
        m_Scene = PhysicsEngineUtils::GetPhysicsEngine()->GetActiveScene();
        if (!m_Scene)
        {
            PhysicsSceneCreateOptions sceneOptions;
            sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);
            sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
            // sceneOptions.bGpuDynamicsEnabled = true;
            // sceneOptions.broadPhaseType = PhysicsBroadPhaseType::GPU;
            // sceneOptions.solverType = PhysicsSolverType::TGS;
            m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);
            PhysicsEngineUtils::GetPhysicsEngine()->SetActiveScene(m_Scene);
        }

        PhysicsMaterialCreateOptions materialOptions;
        materialOptions.m_DynamicFriction = 0.5f;
        materialOptions.m_StaticFriction = 0.5f;
        materialOptions.m_Restitution = 0.6f;
        materialOptions.m_Density = 1.0f;
        m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

        CreateGround();

        for (uint32_t i = 0; i < 5; i++)
            CreateStack();

        m_initialized = true;
    }

    void Update(float deltaTime) override
    {
        if (!m_initialized || m_paused)
            return;

        m_elapsedTime += deltaTime;

        if (m_Scene)
        {
            m_Scene->Tick(deltaTime);
        }
    }

    void Render() override
    {
    }

    void Reset() override
    {
        Cleanup();
        Initialize();
    }

    void Pause() override
    {
        m_paused = true;
    }

    void Resume() override
    {
        m_paused = false;
    }

    void MouseClickCallback(int x, int y, int button, int action, int mods) override
    {
    }

    void KeyBoardCallback(int key, int scancode, int action, int mods) override
    {
        auto renderer = GetRenderer();
        if (key == ' ' && action == 1 && renderer)
        {
            CollisionGeometryCreateOptions options;
            options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            options.m_SphereParams.m_Radius = 2.0f;

            PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

            auto dynamic = TestRigidBody::CreateDynamic(renderer->GetActiveCamera()->GetTransform(),
                geometry,
                renderer->GetActiveCamera()->GetDir() * 75);
            AddObject(dynamic);
        }
        else if ((key == 'B' || key == 'b') && action == 1)
        {
            CreateStack();
        }
    }

private:
    void CreateGround()
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

    void CreateStack()
    {
        MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
        transform.translate(MathLib::HVector3(0, 0, m_StrackZ -= 10.f));
        auto objects = TestRigidBody::CreateBoxStack(transform, 10, 2.f, 100);
        for (auto& object : objects)
        {
            AddObject(object);
        }
    }

private:
    float m_StrackZ = 0.f;
};

