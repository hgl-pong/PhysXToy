#include "Application.h"
#include "Physics/PhysicsCommon.h"
#include "PxPhysicsAPI.h"
#include <Math/GraphicUtils/Camara.h>
#include "TestRigidBodyCreate.h"
#include "RenderObjectAdapter.h"
#include "PhysicsProfilerImGui.h"
#include "ScenePanel.h"
#include "Test/TestSceneManager.h"
#include "Test/TestSceneBase.h"

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

		if (m_Renderer) {
			m_Renderer->Release();
			m_Renderer.reset();
		}
		
		delete this;
	}
	int Run() override
	{
		while (m_Renderer->Tick())
		{
			m_Scene->Tick(1.f / 60.f);
		}
		return 1;
	}

private:
	void _KeyPressEvent(void* eventData);
	void _KeyReleaseEvent(void* eventData);
	void _MousePressEvent(void* eventData);
	void _MouseReleaseEvent(void* eventData);
	void _MouseMoveEvent(void* eventData);
	void _MouseScrollEvent(void* eventData);

	void _InitPhysics(bool interactive);
	void _AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject> &object);
	PhysicsPtr<IPhysicsObject> _CreateDynamic(const MathLib::HTransform3 &t, PhysicsPtr<IColliderGeometry> &geometry, const MathLib::HVector3 &velocity = MathLib::HVector3(0, 0, 0));
	
	// Scene handling methods
	void _InitScenes();
	void _OnSceneChange(const std::string& sceneName);
	std::shared_ptr<TestSceneBase> _CreateDefaultScene();
	std::shared_ptr<TestSceneBase> _CreateParticleScene();
	std::shared_ptr<TestSceneBase> _CreateRigidBodiesScene();
	std::shared_ptr<TestSceneBase> _CreateBoxStackScene();

private:
	PhysicsPtr<IRenderer> m_Renderer;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	PhysicsPtr<IPhysicsScene> m_Scene;
	std::shared_ptr<PhysicsProfilerImGui> m_ProfilerImGui;
	std::shared_ptr<ScenePanel> m_ScenePanel;
	
	size_t m_SceneChangeCallbackId;
};

TestingApplication::TestingApplication(int argc, char **argv)
{
	m_Renderer = make_physics_ptr(CreateRenderer(argc, argv));
	m_Renderer->SetApplicationName("Physics Engine Testing Application");
	m_Renderer->SetUp(
		std::bind(&TestingApplication::_MousePressEvent, this, std::placeholders::_1),
		std::bind(&TestingApplication::_MouseReleaseEvent, this, std::placeholders::_1),
		std::bind(&TestingApplication::_MouseMoveEvent, this, std::placeholders::_1),
		std::bind(&TestingApplication::_MouseScrollEvent, this, std::placeholders::_1),
		std::bind(&TestingApplication::_KeyPressEvent, this, std::placeholders::_1),
		std::bind(&TestingApplication::_KeyReleaseEvent, this, std::placeholders::_1)
	);
	
	// Initialize physics
	_InitPhysics(true);
	
	// Initialize profiler panel
	m_ProfilerImGui = std::make_shared<PhysicsProfilerImGui>(PhysicsEngineUtils::GetProfiler());
	m_Renderer->AddGUIPanel(m_ProfilerImGui);
	
	auto& sceneManager = TestSceneManager::GetInstance();
	m_SceneChangeCallbackId = sceneManager.RegisterSceneChangeCallback(
		std::bind(&TestingApplication::_OnSceneChange, this, std::placeholders::_1)
	);
	
	_InitScenes();
	
	m_ScenePanel = std::make_shared<ScenePanel>();
	m_Renderer->AddGUIPanel(m_ScenePanel);
}

void TestingApplication::_InitScenes()
{
	auto& sceneManager = TestSceneManager::GetInstance();

	sceneManager.RegisterScene("Default Scene", 
		std::bind(&TestingApplication::_CreateDefaultScene, this),
		"Default physics scene with standard gravity and bunny models.");
	
	sceneManager.RegisterScene("Particle System", 
		std::bind(&TestingApplication::_CreateParticleScene, this),
		"Fluid particle simulation with physics interactions.");
	
	sceneManager.RegisterScene("Rigid Bodies", 
		std::bind(&TestingApplication::_CreateRigidBodiesScene, this),
		"Multiple rigid bodies with collision detection and response.");

	sceneManager.RegisterScene("Box Stack", 
		std::bind(&TestingApplication::_CreateBoxStackScene, this),
		"Stack of boxes demonstrating physics stability and constraints.");
}

void TestingApplication::_OnSceneChange(const std::string& sceneName)
{
	if (m_Scene)
	{
		m_Scene.reset();
	}
	
	auto currentScene = TestSceneManager::GetInstance().GetCurrentScene();
	if (currentScene)
	{

	}
}

std::shared_ptr<TestSceneBase> TestingApplication::_CreateDefaultScene()
{
	_InitPhysics(true);
	
	return std::shared_ptr<TestSceneBase>();
}

std::shared_ptr<TestSceneBase> TestingApplication::_CreateParticleScene()
{
	PhysicsEngineOptions options;
	options.m_NumThreads = 10;
	IPhysicsEngine *engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

	PhysicsSceneCreateOptions sceneOptions;
	sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
	sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

	m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);
	
	return std::shared_ptr<TestSceneBase>();
}

std::shared_ptr<TestSceneBase> TestingApplication::_CreateRigidBodiesScene()
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
	
	m_Scene->AddPhysicsObject(groundPlaneObject);
	
	for (int i = 0; i < 10; i++)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 1.0f + (float)i * 0.2f;

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);
		
		MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
		transform.translate(MathLib::HVector3(i * 3.0f, 10.0f + i * 2.0f, 0.0f));
		
		_CreateDynamic(transform, geometry);
	}
	
	return std::shared_ptr<TestSceneBase>();
}

std::shared_ptr<TestSceneBase> TestingApplication::_CreateBoxStackScene()
{
	// 堆叠箱子场景示例
	PhysicsEngineOptions options;
	options.m_NumThreads = 10;
	IPhysicsEngine *engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

	PhysicsSceneCreateOptions sceneOptions;
	sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
	sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

	m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);
	
	// 创建材质
	PhysicsMaterialCreateOptions materialOptions;
	materialOptions.m_StaticFriction = 0.8f; // 更高的摩擦力以获得更好的堆叠效果
	materialOptions.m_DynamicFriction = 0.8f;
	materialOptions.m_Restitution = 0.1f; // 较低的恢复系数减少弹跳
	materialOptions.m_Density = 10.0f;
	
	return std::shared_ptr<TestSceneBase>();
}

void TestingApplication::_KeyPressEvent(void* eventData)
{
	if (!eventData) return;
	
	int key = *reinterpret_cast<int*>(eventData);
}

void TestingApplication::_KeyReleaseEvent(void* eventData)
{
	if (!eventData) return;
	
	int key = *reinterpret_cast<int*>(eventData);
	

	if (key == ' ') {
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 2.0f;

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		_CreateDynamic(m_Renderer->GetActiveCamera()->GetTransform(), geometry, m_Renderer->GetActiveCamera()->GetDir() * 75);
	}

	else if (key == 'B' || key == 'b') {
		auto physicsObject = TestRigidBody::TestRigidBodyCreate();
		if (m_Scene)
		{
			for (auto &physicsObject : physicsObject)
			{
				m_Scene->AddPhysicsObject(physicsObject);
				_AddPhysicsDebugRenderableObject(physicsObject);
			}
		}
	}
}

void TestingApplication::_MousePressEvent(void* eventData)
{
	if (!eventData) return;
	
	int button = *reinterpret_cast<int*>(eventData);
}

void TestingApplication::_MouseReleaseEvent(void* eventData)
{
	if (!eventData) return;
	
	int button = *reinterpret_cast<int*>(eventData);
}

void TestingApplication::_MouseMoveEvent(void* eventData)
{
	if (!eventData) return;
	
	struct MouseMoveData {
		double xpos, ypos, deltaX, deltaY;
	};
	
	MouseMoveData* moveData = reinterpret_cast<MouseMoveData*>(eventData);
}

void TestingApplication::_MouseScrollEvent(void* eventData)
{
	if (!eventData) return;
	
	struct ScrollData {
		double xoffset, yoffset;
	};
	
	ScrollData* scrollData = reinterpret_cast<ScrollData*>(eventData);
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
	 //TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\banana.obj", 1);
	 //TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\armadillo.obj",0.4);
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
	std::shared_ptr<RenderObject> renderable = std::make_shared<RenderObjectAdapter>(physicsObject);
	m_Renderer->AddRenderObject(renderable);
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
