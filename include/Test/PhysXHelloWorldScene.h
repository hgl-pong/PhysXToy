#pragma once

#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"
#include <memory>
#include <vector>

class PhysXHelloWorldScene : public TestSceneBase
{
public:
    PhysXHelloWorldScene() : 
        TestSceneBase("Basic physics scene with a ground plane, sphere, and boxes. As Same as Hello World in PhysX Snippet.")
    {
    }

    ~PhysXHelloWorldScene() override
    {
        Cleanup();
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

        CreateSphere();

        CreateCloth();

        CreateBoxes();

        m_initialized = true;
    }

    void Update(float deltaTime) override
    {
        if (!m_initialized || m_paused)
            return;

        m_elapsedTime += deltaTime;

        // 旋转球体
        if (m_sphere && (m_sphere->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC))
        {
            auto dynamicSphere = std::dynamic_pointer_cast<IRigidDynamic>(m_sphere);
            if (dynamicSphere && !dynamicSphere->IsKinematic())
            {
                dynamicSphere->SetKinematic(true);
            }
        }

        if (m_sphere && (m_sphere->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC))
        {
            auto dynamicSphere = std::dynamic_pointer_cast<IRigidDynamic>(m_sphere);
            if (dynamicSphere && dynamicSphere->IsKinematic())
            {
                const float speed = 2.0f;
                MathLib::HTransform3 pose = m_sphere->GetTransform();
                MathLib::HTransform3 newPose = pose;

                m_sphere->SetTransform(newPose);
            }
        }

        if (m_Scene)
        {
            m_Scene->Tick(deltaTime);
        }
    }

    void Render() override
    {
    }

    void Cleanup() override
    {
        if (m_Scene)
        {
            if (m_ground)
            {
                m_Scene->RemovePhysicsObject(m_ground);
                m_ground.reset();
            }

            if (m_sphere)
            {
                m_Scene->RemovePhysicsObject(m_sphere);
                m_sphere.reset();
            }

            if (m_cloth)
            {
                m_cloth.reset();
            }

            for (auto& box : m_boxes)
            {
                if (box)
                {
                    m_Scene->RemovePhysicsObject(box);
                    box.reset();
                }
            }
            m_boxes.clear();
        }

        m_initialized = false;
    }

    std::string GetName() const override
    {
        return "PhysX Hello World Scene";
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
        if (key == 'P' && action == 1)
        {
            if (m_paused)
                Resume();
            else
                Pause();
        }
    }

private:
    void CreateGround()
    {
        PhysicsObjectCreateOptions options;
        options.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
        
        m_ground = PhysicsEngineUtils::CreateObject(options);
        if (!m_ground)
            return;

        MathLib::HTransform3 transform;
        transform.setIdentity();
        m_ground->SetTransform(transform);

        CollisionGeometryCreateOptions geoOptions;
        geoOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE;
        geoOptions.m_PlaneParams.m_Normal = MathLib::HVector3(0.0f, 1.0f, 0.0f);
        geoOptions.m_PlaneParams.m_Distance = 0.0f;
        
        auto planeGeo = PhysicsEngineUtils::CreateColliderGeometry(geoOptions);
        if (planeGeo)
        {
            m_ground->AddColliderGeometry(planeGeo, MathLib::HTransform3::Identity());
        }

        m_Scene->AddPhysicsObject(m_ground);
    }

    void CreateSphere()
    {
        PhysicsObjectCreateOptions options;
        options.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
        // options.material = m_Material; // 这个成员不存在
        
        m_sphere = PhysicsEngineUtils::CreateObject(options);
        if (!m_sphere)
            return;

        MathLib::HTransform3 transform;
        transform.setIdentity();
        transform.translation() = MathLib::HVector3(0.0f, 5.0f, 0.0f);
        m_sphere->SetTransform(transform);

        CollisionGeometryCreateOptions geoOptions;
        geoOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
        geoOptions.m_SphereParams.m_Radius = 3.0f;
        
        auto sphereGeo = PhysicsEngineUtils::CreateColliderGeometry(geoOptions);
        if (sphereGeo)
        {
            m_sphere->AddColliderGeometry(sphereGeo, MathLib::HTransform3::Identity());
        }

        if (auto dynamicSphere = std::dynamic_pointer_cast<IRigidDynamic>(m_sphere))
        {
            dynamicSphere->SetKinematic(true);
        }

        m_Scene->AddPhysicsObject(m_sphere);
    }

    void CreateCloth()
    {
        const uint32_t numX = 250;
        const uint32_t numZ = 250;
        const float particleSpacing = 0.05f;
        const float totalClothMass = 10.0f;
        const MathLib::HVector3 position(-0.5f * numX * particleSpacing, 8.0f, -0.5f * numZ * particleSpacing);

        ClothCreateOptions clothOptions;
        clothOptions.m_MeshDesc.m_Width = numX;
        clothOptions.m_MeshDesc.m_Height = numZ;
        clothOptions.m_MeshDesc.m_ParticleSpacing = particleSpacing;
        clothOptions.m_Params.m_Mass = totalClothMass;
        clothOptions.m_Transform.translation() = position;
        clothOptions.m_Params.m_Stiffness = 10000.0f;
        clothOptions.m_Params.m_BendingStiffness = 100.0f;
        clothOptions.m_Params.m_Damping = 0.001f;
        
        m_cloth = PhysicsEngineUtils::GetPhysicsEngine()->CreateCloth(clothOptions);
        
        // if (m_cloth && m_Scene)
        // {
        //     m_Scene->AddCloth(m_cloth);
        // }
    }

    void CreateBoxes()
    {
        const float boxSize = 1.0f;
        const float boxMass = 1.0f;

        CollisionGeometryCreateOptions geoOptions;
        geoOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
        geoOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(0.5f * boxSize, 0.5f * boxSize, 0.5f * boxSize);
        
        auto boxGeo = PhysicsEngineUtils::CreateColliderGeometry(geoOptions);
        if (!boxGeo)
            return;

        for (int i = 0; i < 5; ++i)
        {
            PhysicsObjectCreateOptions options;
            options.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
            
            auto box = PhysicsEngineUtils::CreateObject(options);
            if (!box)
                continue;

            MathLib::HTransform3 transform;
            transform.setIdentity();
            transform.translation() = MathLib::HVector3(i - 3.0f, 10.0f, 4.0f);
            box->SetTransform(transform);

            box->AddColliderGeometry(boxGeo, MathLib::HTransform3::Identity());

            if (auto dynamicBox = std::dynamic_pointer_cast<IRigidDynamic>(box))
            {
                dynamicBox->SetMass(boxMass);
            }

            m_Scene->AddPhysicsObject(box);
            m_boxes.push_back(box);
        }
    }

private:
    PhysicsPtr<IPhysicsScene> m_Scene;
    PhysicsPtr<IPhysicsMaterial> m_Material;
    PhysicsPtr<IPhysicsObject> m_ground;
    PhysicsPtr<IPhysicsObject> m_sphere;
    PhysicsPtr<ICloth> m_cloth;
    std::vector<PhysicsPtr<IPhysicsObject>> m_boxes;
};

