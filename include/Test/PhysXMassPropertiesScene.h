#pragma once
#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"
#include <memory>
#include <vector>
#include "TestRigidBodyCreate.h"
#include <Math/GraphicUtils/Camara.h>

class PhysXMassPropertiesScene : public TestSceneBase
{
public:
    PhysXMassPropertiesScene() : 
        TestSceneBase(TestSceneType::PHYSX_MASS_PROPERTIES)
    {
    }

    ~PhysXMassPropertiesScene() override
    {
    }

    void Initialize() override
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
        CreateSnowMen();

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
            // Shoot a sphere to test the mass properties of the snowman
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

    PhysicsPtr<IPhysicsObject> CreateSnowMan(const MathLib::HTransform3& pos, int mode)
    {
        PhysicsObjectCreateOptions objectOptions;
        objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
        objectOptions.m_Transform = pos;
        PhysicsPtr<IPhysicsObject> snowmanActor = PhysicsEngineUtils::CreateObject(objectOptions);
        IRigidDynamic* snowmanDynamic = dynamic_cast<IRigidDynamic*>(snowmanActor.get());
        
        if (!snowmanActor)
        {
            return nullptr;
        }

        PhysicsPtr<IColliderGeometry> armL = nullptr;
        PhysicsPtr<IColliderGeometry> armR = nullptr;

        switch (mode % 5)
        {
        case 0: // The snowman with the bottom weight
        {
            // The hidden weight sphere at the bottom
            CollisionGeometryCreateOptions weightOptions;
            weightOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            weightOptions.m_SphereParams.m_Radius = 0.2f;
            PhysicsPtr<IColliderGeometry> weightShape = PhysicsEngineUtils::CreateColliderGeometry(weightOptions);
            MathLib::HTransform3 weightTransform(MathLib::HTranslation3(MathLib::HVector3(0, -0.29f, 0)));
            snowmanActor->AddColliderGeometry(weightShape, weightTransform);
            
            // Update the mass of the bottom weight separately
            if (snowmanDynamic)
                snowmanDynamic->SetMass(10.0f);
            
            // The bottom sphere of the snowman
            CollisionGeometryCreateOptions baseOptions;
            baseOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            baseOptions.m_SphereParams.m_Radius = 0.5f;
            PhysicsPtr<IColliderGeometry> baseShape = PhysicsEngineUtils::CreateColliderGeometry(baseOptions);
            snowmanActor->AddColliderGeometry(baseShape, MathLib::HTransform3::Identity());
            
            // The middle sphere of the snowman
            CollisionGeometryCreateOptions middleOptions;
            middleOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            middleOptions.m_SphereParams.m_Radius = 0.4f;
            PhysicsPtr<IColliderGeometry> middleShape = PhysicsEngineUtils::CreateColliderGeometry(middleOptions);
            MathLib::HTransform3 middleTransform(MathLib::HTranslation3(MathLib::HVector3(0, 0.6f, 0)));
            snowmanActor->AddColliderGeometry(middleShape, middleTransform);
            
            // The head sphere of the snowman
            CollisionGeometryCreateOptions headOptions;
            headOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            headOptions.m_SphereParams.m_Radius = 0.3f;
            PhysicsPtr<IColliderGeometry> headShape = PhysicsEngineUtils::CreateColliderGeometry(headOptions);
            MathLib::HTransform3 headTransform(MathLib::HTranslation3(MathLib::HVector3(0, 1.1f, 0)));
            snowmanActor->AddColliderGeometry(headShape, headTransform);
            
            // The left arm
            CollisionGeometryCreateOptions armLOptions;
            armLOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armLOptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armLOptions.m_CapsuleParams.m_Radius = 0.1f;
            armL = PhysicsEngineUtils::CreateColliderGeometry(armLOptions);
            MathLib::HTransform3 armLTransform(MathLib::HTranslation3(MathLib::HVector3(-0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armL, armLTransform);
            
            // The right arm
            CollisionGeometryCreateOptions armROptions;
            armROptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armROptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armROptions.m_CapsuleParams.m_Radius = 0.1f;
            armR = PhysicsEngineUtils::CreateColliderGeometry(armROptions);
            MathLib::HTransform3 armRTransform(MathLib::HTranslation3(MathLib::HVector3(0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armR, armRTransform);
        }
        break;
        
        case 1: // Only consider the mass of the bottom sphere
        {
            // The bottom sphere of the snowman
            CollisionGeometryCreateOptions baseOptions;
            baseOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            baseOptions.m_SphereParams.m_Radius = 0.5f;
            PhysicsPtr<IColliderGeometry> baseShape = PhysicsEngineUtils::CreateColliderGeometry(baseOptions);
            snowmanActor->AddColliderGeometry(baseShape, MathLib::HTransform3::Identity());
            
            // Update the mass of the bottom sphere separately
            if (snowmanDynamic)
                snowmanDynamic->SetMass(1.0f);
            
            // The middle sphere of the snowman
            CollisionGeometryCreateOptions middleOptions;
            middleOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            middleOptions.m_SphereParams.m_Radius = 0.4f;
            PhysicsPtr<IColliderGeometry> middleShape = PhysicsEngineUtils::CreateColliderGeometry(middleOptions);
            MathLib::HTransform3 middleTransform(MathLib::HTranslation3(MathLib::HVector3(0, 0.6f, 0)));
            snowmanActor->AddColliderGeometry(middleShape, middleTransform);
            
            // The head sphere of the snowman
            CollisionGeometryCreateOptions headOptions;
            headOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            headOptions.m_SphereParams.m_Radius = 0.3f;
            PhysicsPtr<IColliderGeometry> headShape = PhysicsEngineUtils::CreateColliderGeometry(headOptions);
            MathLib::HTransform3 headTransform(MathLib::HTranslation3(MathLib::HVector3(0, 1.1f, 0)));
            snowmanActor->AddColliderGeometry(headShape, headTransform);
            
            // The left arm
            CollisionGeometryCreateOptions armLOptions;
            armLOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armLOptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armLOptions.m_CapsuleParams.m_Radius = 0.1f;
            armL = PhysicsEngineUtils::CreateColliderGeometry(armLOptions);
            MathLib::HTransform3 armLTransform(MathLib::HTranslation3(MathLib::HVector3(-0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armL, armLTransform);
            
            // The right arm
            CollisionGeometryCreateOptions armROptions;
            armROptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armROptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armROptions.m_CapsuleParams.m_Radius = 0.1f;
            armR = PhysicsEngineUtils::CreateColliderGeometry(armROptions);
            MathLib::HTransform3 armRTransform(MathLib::HTranslation3(MathLib::HVector3(0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armR, armRTransform);
            
            // Set the mass center position
            if (snowmanDynamic)
                snowmanDynamic->SetCenterOfMass(MathLib::HVector3(0, -0.5f, 0));
        }
        break;
        
        case 2: // Consider the overall mass
        {
            // The bottom sphere of the snowman
            CollisionGeometryCreateOptions baseOptions;
            baseOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            baseOptions.m_SphereParams.m_Radius = 0.5f;
            PhysicsPtr<IColliderGeometry> baseShape = PhysicsEngineUtils::CreateColliderGeometry(baseOptions);
            snowmanActor->AddColliderGeometry(baseShape, MathLib::HTransform3::Identity());
            
            // The middle sphere of the snowman
            CollisionGeometryCreateOptions middleOptions;
            middleOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            middleOptions.m_SphereParams.m_Radius = 0.4f;
            PhysicsPtr<IColliderGeometry> middleShape = PhysicsEngineUtils::CreateColliderGeometry(middleOptions);
            MathLib::HTransform3 middleTransform(MathLib::HTranslation3(MathLib::HVector3(0, 0.6f, 0)));
            snowmanActor->AddColliderGeometry(middleShape, middleTransform);
            
            // The head sphere of the snowman
            CollisionGeometryCreateOptions headOptions;
            headOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            headOptions.m_SphereParams.m_Radius = 0.3f;
            PhysicsPtr<IColliderGeometry> headShape = PhysicsEngineUtils::CreateColliderGeometry(headOptions);
            MathLib::HTransform3 headTransform(MathLib::HTranslation3(MathLib::HVector3(0, 1.1f, 0)));
            snowmanActor->AddColliderGeometry(headShape, headTransform);
            
            // The left arm
            CollisionGeometryCreateOptions armLOptions;
            armLOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armLOptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armLOptions.m_CapsuleParams.m_Radius = 0.1f;
            armL = PhysicsEngineUtils::CreateColliderGeometry(armLOptions);
            MathLib::HTransform3 armLTransform(MathLib::HTranslation3(MathLib::HVector3(-0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armL, armLTransform);
            
            // The right arm
            CollisionGeometryCreateOptions armROptions;
            armROptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armROptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armROptions.m_CapsuleParams.m_Radius = 0.1f;
            armR = PhysicsEngineUtils::CreateColliderGeometry(armROptions);
            MathLib::HTransform3 armRTransform(MathLib::HTranslation3(MathLib::HVector3(0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armR, armRTransform);
            
            // After adding the mass of all parts, set the mass center position
            if (snowmanDynamic) {
                snowmanDynamic->SetMass(1.0f);
                snowmanDynamic->SetCenterOfMass(MathLib::HVector3(0, -0.5f, 0));
            }
        }
        break;
        
        case 3: // Low mass center
        {
            // The bottom sphere of the snowman
            CollisionGeometryCreateOptions baseOptions;
            baseOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            baseOptions.m_SphereParams.m_Radius = 0.5f;
            PhysicsPtr<IColliderGeometry> baseShape = PhysicsEngineUtils::CreateColliderGeometry(baseOptions);
            snowmanActor->AddColliderGeometry(baseShape, MathLib::HTransform3::Identity());
            
            // The middle sphere of the snowman
            CollisionGeometryCreateOptions middleOptions;
            middleOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            middleOptions.m_SphereParams.m_Radius = 0.4f;
            PhysicsPtr<IColliderGeometry> middleShape = PhysicsEngineUtils::CreateColliderGeometry(middleOptions);
            MathLib::HTransform3 middleTransform(MathLib::HTranslation3(MathLib::HVector3(0, 0.6f, 0)));
            snowmanActor->AddColliderGeometry(middleShape, middleTransform);
            
            // The head sphere of the snowman
            CollisionGeometryCreateOptions headOptions;
            headOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            headOptions.m_SphereParams.m_Radius = 0.3f;
            PhysicsPtr<IColliderGeometry> headShape = PhysicsEngineUtils::CreateColliderGeometry(headOptions);
            MathLib::HTransform3 headTransform(MathLib::HTranslation3(MathLib::HVector3(0, 1.1f, 0)));
            snowmanActor->AddColliderGeometry(headShape, headTransform);
            
            // The left arm
            CollisionGeometryCreateOptions armLOptions;
            armLOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armLOptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armLOptions.m_CapsuleParams.m_Radius = 0.1f;
            armL = PhysicsEngineUtils::CreateColliderGeometry(armLOptions);
            MathLib::HTransform3 armLTransform(MathLib::HTranslation3(MathLib::HVector3(-0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armL, armLTransform);
            
            // The right arm
            CollisionGeometryCreateOptions armROptions;
            armROptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armROptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armROptions.m_CapsuleParams.m_Radius = 0.1f;
            armR = PhysicsEngineUtils::CreateColliderGeometry(armROptions);
            MathLib::HTransform3 armRTransform(MathLib::HTranslation3(MathLib::HVector3(0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armR, armRTransform);
            
            // Update the mass and inertia using the specified local position
            if (snowmanDynamic) {
                MathLib::HVector3 localPos(0, -0.5f, 0);
                snowmanDynamic->SetMass(1.0f);
                snowmanDynamic->SetCenterOfMass(localPos);
            }
        }
        break;
        
        case 4: // Manually set the mass properties
        {
            // The bottom sphere of the snowman
            CollisionGeometryCreateOptions baseOptions;
            baseOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            baseOptions.m_SphereParams.m_Radius = 0.5f;
            PhysicsPtr<IColliderGeometry> baseShape = PhysicsEngineUtils::CreateColliderGeometry(baseOptions);
            snowmanActor->AddColliderGeometry(baseShape, MathLib::HTransform3::Identity());
            
            // The middle sphere of the snowman
            CollisionGeometryCreateOptions middleOptions;
            middleOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            middleOptions.m_SphereParams.m_Radius = 0.4f;
            PhysicsPtr<IColliderGeometry> middleShape = PhysicsEngineUtils::CreateColliderGeometry(middleOptions);
            MathLib::HTransform3 middleTransform(MathLib::HTranslation3(MathLib::HVector3(0, 0.6f, 0)));
            snowmanActor->AddColliderGeometry(middleShape, middleTransform);
            
            // The head sphere of the snowman
            CollisionGeometryCreateOptions headOptions;
            headOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
            headOptions.m_SphereParams.m_Radius = 0.3f;
            PhysicsPtr<IColliderGeometry> headShape = PhysicsEngineUtils::CreateColliderGeometry(headOptions);
            MathLib::HTransform3 headTransform(MathLib::HTranslation3(MathLib::HVector3(0, 1.1f, 0)));
            snowmanActor->AddColliderGeometry(headShape, headTransform);
            
            // The left arm
            CollisionGeometryCreateOptions armLOptions;
            armLOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armLOptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armLOptions.m_CapsuleParams.m_Radius = 0.1f;
            armL = PhysicsEngineUtils::CreateColliderGeometry(armLOptions);
            MathLib::HTransform3 armLTransform(MathLib::HTranslation3(MathLib::HVector3(-0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armL, armLTransform);
            
            // The right arm
            CollisionGeometryCreateOptions armROptions;
            armROptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
            armROptions.m_CapsuleParams.m_HalfHeight = 0.1f;
            armROptions.m_CapsuleParams.m_Radius = 0.1f;
            armR = PhysicsEngineUtils::CreateColliderGeometry(armROptions);
            MathLib::HTransform3 armRTransform(MathLib::HTranslation3(MathLib::HVector3(0.4f, 0.7f, 0)));
            snowmanActor->AddColliderGeometry(armR, armRTransform);
            
            // Manually set the mass properties
            if (snowmanDynamic) {
                snowmanDynamic->SetMass(1.0f);
                snowmanDynamic->SetCenterOfMass(MathLib::HVector3(0, -0.5f, 0));
                MathLib::HMatrix3 inertiaTensor = MathLib::HMatrix3::Identity();
                inertiaTensor(0,0) = 0.05f;
                inertiaTensor(1,1) = 100.0f;
                inertiaTensor(2,2) = 100.0f;
                snowmanDynamic->SetMassProperties(1.0f, MathLib::HVector3(0, -0.5f, 0), inertiaTensor);
            }
        }
        break;
        }
        
        return snowmanActor;
    }

    void CreateSnowMen()
    {
        uint32_t numSnowmen = 5;
        for (uint32_t i = 0; i < numSnowmen; i++)
        {
            MathLib::HVector3 pos(i * 2.5f, 1.0f, -8.0f);
            MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
            transform.translate(pos);
            PhysicsPtr<IPhysicsObject> snowman = CreateSnowMan(transform, i);
            if (snowman) {
                AddObject(snowman);
            }
        }
    }
}; 