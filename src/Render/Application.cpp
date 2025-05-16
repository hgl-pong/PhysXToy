#include "Application.h"
#include "Physics/PhysicsCommon.h"
#include <Math/GraphicUtils/Camara.h>
#include "RenderObjectAdapter.h"
#include "PhysicsProfilerImGui.h"
#include "ScenePanel.h"
#include "Test/TestSceneManager.h"
#include "Test/TestSceneBase.h"
#include "Test/TestScene.h"

PhysicsEngineTestingApplication *pApp = nullptr;

class TestingApplication : public PhysicsEngineTestingApplication
{
public:
	explicit TestingApplication(int argc, char **argv);

public:
	void Release() override
	{
		if (m_Renderer) {
			m_Renderer->RemoveGUIPanel(m_ScenePanel);
		}
		m_ScenePanel.reset();
		TestSceneManager::DestroyInstance();
		PhysicsEngineUtils::DestroyPhysicsEngine();

		if (m_Renderer) {
			m_Renderer->Release();
			m_Renderer.reset();
		}
		
		delete this;
	}
	int Run() override
	{
		auto& sceneManager = TestSceneManager::GetInstance();
		
		while (m_Renderer->Tick())
		{
			sceneManager.UpdateCurrentScene(1.f / 60.f);
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

private:
	PhysicsPtr<IRenderer> m_Renderer;
	std::shared_ptr<PhysicsProfilerImGui> m_ProfilerImGui;
	std::shared_ptr<ScenePanel> m_ScenePanel;
};

TestingApplication::TestingApplication(int argc, char **argv)
{
	PhysicsEngineOptions options;
	options.m_NumThreads = 10;
	IPhysicsEngine* engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

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
	
	m_ProfilerImGui = std::make_shared<PhysicsProfilerImGui>(PhysicsEngineUtils::GetProfiler());
	m_Renderer->AddGUIPanel(m_ProfilerImGui);
	
	m_ScenePanel = std::make_shared<ScenePanel>();
	m_Renderer->AddGUIPanel(m_ScenePanel);

}

void TestingApplication::_KeyPressEvent(void* eventData)
{
	if (!eventData) return;
	
	int key = *reinterpret_cast<int*>(eventData);

	auto currentScene = TestSceneManager::GetInstance().GetCurrentScene();
	if (currentScene)
	{
		currentScene->KeyBoardCallback(key, 0, 1, 0);
	}
}

void TestingApplication::_KeyReleaseEvent(void* eventData)
{
	if (!eventData) return;
	
	int key = *reinterpret_cast<int*>(eventData);
	
	auto currentScene = TestSceneManager::GetInstance().GetCurrentScene();
	if (currentScene)
	{
		currentScene->KeyBoardCallback(key, 0, 0, 0);
	}
}

void TestingApplication::_MousePressEvent(void* eventData)
{
	if (!eventData) return;
	
	int button = *reinterpret_cast<int*>(eventData);
	
	auto currentScene = TestSceneManager::GetInstance().GetCurrentScene();
	if (currentScene)
	{
		currentScene->MouseClickCallback(0, 0, button, 1, 0);
	}
}

void TestingApplication::_MouseReleaseEvent(void* eventData)
{
	if (!eventData) return;
	
	int button = *reinterpret_cast<int*>(eventData);
	
	auto currentScene = TestSceneManager::GetInstance().GetCurrentScene();
	if (currentScene)
	{
		currentScene->MouseClickCallback(0, 0, button, 0, 0);
	}
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
