#include "Test/TestSceneManager.h"

#include "Test/PhysXHelloWorldScene.h"
#include "Test/TestScene.h"
#include "Test/MassPropertiesScene.h"

#include <algorithm>

// Singleton instance
TestSceneManager& TestSceneManager::GetInstance()
{
    static TestSceneManager instance;
    return instance;
}

bool TestSceneManager::SwitchToScene(TestSceneType type)
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    // Don't switch if it's already the current scene
    if (m_CurrentSceneType == type && m_CurrentScene != nullptr)
    {
        return true;
    }
    
    // Cleanup the current scene
    CleanupCurrentScene();
    
    // Create the new scene
    m_CurrentSceneType = type;
    m_CurrentScene = CreateTestScene(type);
    
    // Initialize the new scene
    if (m_CurrentScene)
    {
        m_CurrentScene->Initialize();
        return true;
    }
    
    // If we get here, scene creation failed
    m_CurrentSceneType = TestSceneType::TEST_SCENE_COUNT; // Invalid scene type
    return false;
}

std::shared_ptr<TestSceneBase> TestSceneManager::GetCurrentScene() const
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    return m_CurrentScene;
}

void TestSceneManager::UpdateCurrentScene(float deltaTime)
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    if (m_CurrentScene)
    {
        m_CurrentScene->Update(deltaTime);
    }
}

void TestSceneManager::RenderCurrentScene()
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    if (m_CurrentScene)
    {
        m_CurrentScene->Render();
    }
}

void TestSceneManager::ResetCurrentScene()
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    if (m_CurrentScene)
    {
        m_CurrentScene->Reset();
    }
}

void TestSceneManager::PauseCurrentScene()
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    if (m_CurrentScene)
    {
        m_CurrentScene->Pause();
    }
}

void TestSceneManager::ResumeCurrentScene()
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    if (m_CurrentScene)
    {
        m_CurrentScene->Resume();
    }
}

bool TestSceneManager::IsCurrentScenePause() const
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    if(m_CurrentScene)
    {
        return m_CurrentScene->IsPaused();
    }
    return true;
}

size_t TestSceneManager::GetSceneCount() const
{
    return static_cast<size_t>(TestSceneType::TEST_SCENE_COUNT);
}

std::string TestSceneManager::GetSceneDescription() const
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    if(m_CurrentScene)
    {
        return m_CurrentScene->GetDescription();
    }
    return "";
}

void TestSceneManager::CleanupCurrentScene()
{
    // No need for lock, already locked by caller
    
    if (m_CurrentScene)
    {
        m_CurrentScene->Cleanup();
        m_CurrentScene.reset();
    }
}

std::shared_ptr<TestSceneBase> CreateTestScene(TestSceneType type)
{
    switch (type)
    {
    case TestSceneType::DEFAULT_SCENE:
        return std::make_shared<TestScene>();
    case TestSceneType::PHYSX_HELLO_WORLD:
        return std::make_shared<PhysXHelloWorldScene>();
    case TestSceneType::PHYSX_MASS_PROPERTIES:
        return std::make_shared<MassPropertiesScene>();
    default:
        return nullptr;
    } 
}