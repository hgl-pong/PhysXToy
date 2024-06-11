#include "Application.h"
#include "MagnumRender/MagnumRenderCommon.h"
#include "Physics/PhysicsCommon.h"
#include "PxPhysicsAPI.h"

#include "TestRigidBodyCreate.h"
#include "PhysicsRenderObject.h"

using namespace physx;
PhysicsEngineTestingApplication *pApp = nullptr;

class TestingApplication : public PhysicsEngineTestingApplication
{
public:
	explicit TestingApplication(int argc, char **argv);

public:
	void Release() override
	{
		m_Scene.reset();
		m_Material.reset();
		PhysicsEngineUtils::DestroyPhysicsEngine();
		delete this;
	}
	int Run() override
	{
		while (true)
		{
			m_Scene->Tick(1.f / 60.f);
			m_Renderer->Tick();
		}
		return 1;
	}

private:
	void _KeyPressEvent(Magnum::Platform::Sdl2Application::KeyEvent &event);
	void _KeyReleaseEvent(Magnum::Platform::Sdl2Application::KeyEvent &event);
	void _MousePressEvent(Magnum::Platform::Sdl2Application::MouseEvent &event);
	void _MouseMoveEvent(Magnum::Platform::Sdl2Application::MouseMoveEvent &event);
	void _MouseScrollEvent(Magnum::Platform::Sdl2Application::MouseScrollEvent &event);

	void _InitPhysics(bool interactive);
	void _AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject> &object);
	PhysicsPtr<IPhysicsObject> _CreateDynamic(const MathLib::HTransform3 &t, PhysicsPtr<IColliderGeometry> &geometry, const MathLib::HVector3 &velocity = MathLib::HVector3(0, 0, 0));

private:
	PhysicsPtr<MagnumRender::Renderer> m_Renderer;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	PhysicsPtr<IPhysicsScene> m_Scene;
};

TestingApplication::TestingApplication(int argc, char **argv)
{
	m_Renderer = make_physics_ptr(MagnumRender::CreateRenderer(argc, argv));
	m_Renderer->SetApplicationName("Physics Engine Testing Application");
	m_Renderer->SetUp(std::bind(&TestingApplication::_MousePressEvent, this, std::placeholders::_1),
					  std::bind(&TestingApplication::_MousePressEvent, this, std::placeholders::_1),
					  std::bind(&TestingApplication::_MouseMoveEvent, this, std::placeholders::_1),
					  std::bind(&TestingApplication::_MouseScrollEvent, this, std::placeholders::_1),
					  std::bind(&TestingApplication::_KeyPressEvent, this, std::placeholders::_1),
					  std::bind(&TestingApplication::_KeyReleaseEvent, this, std::placeholders::_1));
	_InitPhysics(true);
}

void TestingApplication::_KeyPressEvent(Magnum::Platform::Sdl2Application::KeyEvent &event)
{
}

void TestingApplication::_KeyReleaseEvent(Magnum::Platform::Sdl2Application::KeyEvent &event)
{
	char cameraKey = '\0';
	switch (event.key())
	{
	case Magnum::Platform::Sdl2Application::KeyEvent::Key::Space:
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 2.0f;

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		_CreateDynamic(m_Renderer->GetActiveCamera()->GetTransform(), geometry, m_Renderer->GetActiveCamera()->GetDir() * 75);
		break;
	}
	case Magnum::Platform::Sdl2Application::KeyEvent::Key::B:
	{
		auto physicsObject = TestRigidBody::TestRigidBodyCreate();
		if (m_Scene)
		{
			for (auto &physicsObject : physicsObject)
			{
				m_Scene->AddPhysicsObject(physicsObject);
				_AddPhysicsDebugRenderableObject(physicsObject);
			}
		}
		break;
	}
	default:
		break;
	}
}

void TestingApplication::_MousePressEvent(Magnum::Platform::Sdl2Application::MouseEvent &event)
{
}

void TestingApplication::_MouseMoveEvent(Magnum::Platform::Sdl2Application::MouseMoveEvent &event)
{
}

void TestingApplication::_MouseScrollEvent(Magnum::Platform::Sdl2Application::MouseScrollEvent &event)
{
}

void TestingApplication::_InitPhysics(bool interactive)
{
	PhysicsEngineOptions options;
	options.m_NumThreads = 10;
	IPhysicsEngine *engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

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
		m_Scene->AddPhysicsObject(groundPlaneObject);
	//_AddPhysicsDebugRenderableObject(groundPlaneObject);

	TestRigidBody::CreateTestingMeshData(); // Bunny
	// TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\teapot.obj", 0.2);
	// TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\banana.obj", 1);
	// TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\armadillo.obj",0.4);
	auto physicsObject = TestRigidBody::TestRigidBodyCreate();
	if (m_Scene)
	{
		for (auto &physicsObject : physicsObject)
		{
			m_Scene->AddPhysicsObject(physicsObject);
			_AddPhysicsDebugRenderableObject(physicsObject);
		}
	}

	if (!interactive)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 10.0f;
		options.m_Scale = MathLib::HVector3(1.0f, 1.0f, 1.0f);

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		MathLib::HVector3 translation(0, 40, 100);
		MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
		transform.translate(translation);
		_CreateDynamic(transform, geometry, MathLib::HVector3(0, -50, -100));
	}
}

void TestingApplication::_AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject> &physicsObject)
{
	std::shared_ptr<MagnumRender::RenderObject> renderable = std::make_shared<MagnumRender::PhysicsRenderObject>(physicsObject);
	m_Renderer->AddRenderObject(renderable);
	// renderable->UseWorldBoundingBox(true);
	// renderable->ShowBoundingBox(false);
}

PhysicsPtr<IPhysicsObject> TestingApplication::_CreateDynamic(const MathLib::HTransform3 &t, PhysicsPtr<IColliderGeometry> &geometry, const MathLib::HVector3 &velocity)
{
	auto dynamic = TestRigidBody::CreateDynamic(t, geometry, velocity);
	if (m_Scene)
		m_Scene->AddPhysicsObject(dynamic);
	_AddPhysicsDebugRenderableObject(dynamic);
	return dynamic;
}

PhysicsEngineTestingApplication *CreatePhysicsEngineTestingApplication(int argc, char **argv)
{
	if (pApp != nullptr)
		return pApp;
	pApp = new TestingApplication(argc, argv);
	return pApp;
}

PhysicsEngineTestingApplication *GetPhysicsEngineTestingApplication()
{
	return pApp;
}
