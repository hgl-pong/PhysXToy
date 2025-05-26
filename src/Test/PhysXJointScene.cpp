#include "Test/PhysXJointScene.h"
#include "TestRigidBodyCreate.h"
#include "../Render/RenderObjectAdapter.h"
#include "Renderer/Renderer.h"
#include <Math/GraphicUtils/Camara.h>
#include <string>

PhysXJointScene::PhysXJointScene() 
    : TestSceneBase(TestSceneType::PHYSX_JOINT_SCENE)
{
}

PhysXJointScene::~PhysXJointScene()
{
}

void PhysXJointScene::Initialize()
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

    MathLib::HVector3 boxSize(2.0f, 0.5f, 0.5f);

    CollisionGeometryCreateOptions options;
    options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
    options.m_BoxParams.m_HalfExtents = boxSize;

    PhysicsPtr<IColliderGeometry> box = PhysicsEngineUtils::CreateColliderGeometry(options);

    MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
    transform.translate(MathLib::HVector3(0.0f, 20.0f, 0.0f));
	CreateChain(transform, 5, box, 4.0f, std::bind(&PhysXJointScene::CreateLimitedSphericalJoint, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	
    transform.translate(MathLib::HVector3(0.0f, 0.0f, -10.0f));
    CreateChain(transform, 5, box, 4.0f, std::bind(&PhysXJointScene::CreateBreakableFixedJoint, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	
    transform.translate(MathLib::HVector3(0.0f, 0.0f, -10.0f));
    CreateChain(transform, 5, box, 4.0f, std::bind(&PhysXJointScene::CreateDampedD6Joint, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    
    m_initialized = true;
}

void PhysXJointScene::Update(float deltaTime)
{
    if (!m_initialized || m_paused)
        return;

    m_elapsedTime += deltaTime;

    if (m_Scene)
    {
        m_Scene->Tick(deltaTime);
    }
}

void PhysXJointScene::Render()
{
}

void PhysXJointScene::Reset()
{
    Cleanup();
    Initialize();
}

void PhysXJointScene::Pause()
{
    m_paused = true;
}

void PhysXJointScene::Resume()
{
    m_paused = false;
}

void PhysXJointScene::MouseClickCallback(int x, int y, int button, int action, int mods)
{
}

void PhysXJointScene::KeyBoardCallback(int key, int scancode, int action, int mods)
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
}

void PhysXJointScene::CreateGround()
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

void PhysXJointScene::CreateChain(const MathLib::HTransform3& t, uint32_t length, PhysicsPtr<IColliderGeometry>& geometry, MathLib::HReal separation, JointCreateFunction createJoint)
{
	MathLib::HVector3 offset(separation/2, 0, 0);
    MathLib::HTransform3 localTm = MathLib::HTransform3::Identity();

	PhysicsPtr<IPhysicsObject> prev = nullptr;
    PhysicsObjectCreateOptions objOptions;
	for(uint32_t i=0;i<length;i++)
	{
        objOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
        objOptions.m_Transform = t * localTm;
        objOptions.m_IsKinematic = i == 0;
        objOptions.m_MaterialOptions.m_Density = 1.f;
		PhysicsPtr<IPhysicsObject> current = PhysicsEngineUtils::CreateObject(objOptions);
        current->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
        MathLib::HTransform3 localA = MathLib::HTransform3::Identity();
        localA.translate(offset);
        MathLib::HTransform3 localB = MathLib::HTransform3::Identity();
        localB.translate(-offset);
        if(i != 0)
            createJoint(prev, prev ? localA : t, current, localB);
        AddObject(current);
		prev = current;
		localTm.translate(MathLib::HVector3(separation, 0.f, 0.f));
	}
}

PhysicsPtr<IPhysicsJoint> PhysXJointScene::CreateLimitedSphericalJoint(PhysicsPtr<IPhysicsObject> objA, const MathLib::HTransform3& localA,
                                                     PhysicsPtr<IPhysicsObject> objB, const MathLib::HTransform3& localB)
{
    JointCreateOptions options;
    options.type = JointType::SPHERICAL;
    options.objectA = objA;
    options.objectB = objB;
    options.localFrameA = localA;
    options.localFrameB = localB;
    options.collisionEnabled = false;
    
    PhysicsPtr<IPhysicsJoint> joint = PhysicsEngineUtils::CreateJoint(options);
    
    JointLimitOptions limitOptions;
    limitOptions.m_Swing1.m_IsLimited = true;
    limitOptions.m_Swing1.m_UpperLimit = MathLib::H_PI / 4;

    limitOptions.m_Swing2.m_IsLimited = true;
    limitOptions.m_Swing2.m_UpperLimit = MathLib::H_PI / 4;

    joint->SetJointLimits(limitOptions);

    if (m_Scene) {
        m_Scene->AddJoint(joint);
    }
    
    return joint;
}

PhysicsPtr<IPhysicsJoint> PhysXJointScene::CreateBreakableFixedJoint(PhysicsPtr<IPhysicsObject> objA, const MathLib::HTransform3& localA,
                                                  PhysicsPtr<IPhysicsObject> objB, const MathLib::HTransform3& localB)
{
    JointCreateOptions options;
    options.type = JointType::FIXED;
    options.objectA = objA;
    options.objectB = objB;
    options.localFrameA = localA;
    options.localFrameB = localB;
    options.collisionEnabled = false;
    
    PhysicsPtr<IPhysicsJoint> joint = PhysicsEngineUtils::CreateJoint(options);
    
    joint->SetBreakForce(1000.0f);
    joint->SetBreakTorque(100000.0f);
    
    JointLimitOptions limitOptions;
    joint->SetJointLimits(limitOptions);

    if (m_Scene) {
        m_Scene->AddJoint(joint);
    }
    
    return joint;
}

PhysicsPtr<IPhysicsJoint> PhysXJointScene::CreateDampedD6Joint(PhysicsPtr<IPhysicsObject> objA, const MathLib::HTransform3& localA,
                                              PhysicsPtr<IPhysicsObject> objB, const MathLib::HTransform3& localB)
{
    JointCreateOptions options;
    options.type = JointType::D6;
    options.objectA = objA;
    options.objectB = objB;
    options.localFrameA = localA;
    options.localFrameB = localB;
    options.collisionEnabled = false;
    
    PhysicsPtr<IPhysicsJoint> joint = PhysicsEngineUtils::CreateJoint(options);
    
    JointLimitOptions limitOptions;

    limitOptions.m_XAxis.m_IsLimited = true;
    limitOptions.m_YAxis.m_IsLimited = true;
    limitOptions.m_ZAxis.m_IsLimited = true;

    limitOptions.m_Swing1.m_IsLimited = false;
    limitOptions.m_Swing2.m_IsLimited = false;
    limitOptions.m_Twist.m_IsLimited = false;

    joint->SetJointLimits(limitOptions);

    JointDriveSettings driveConfig;
    driveConfig.m_Enabled = true;
    driveConfig.m_Stiffness = 0;
    driveConfig.m_Damping = 1000;
    driveConfig.m_ForceLimit = FLT_MAX;
    driveConfig.m_IsAcceleration = true;
    joint->SetDrive(JointAxis::SLERP, driveConfig);
    if (m_Scene) {
        m_Scene->AddJoint(joint);
    }
    
    return joint;
} 