#include "Test/TestSceneManager.h"

#include "Test/PhysXHelloWorldScene.h"

#include <algorithm>

// Singleton instance
TestSceneManager& TestSceneManager::GetInstance()
{
    static TestSceneManager instance;
    return instance;
}

bool TestSceneManager::RegisterScene(const std::string& name, 
                                   std::function<std::shared_ptr<TestSceneBase>()> factory,
                                   const std::string& description)
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    // Check if scene with this name already exists
    if (m_SceneNameToIndex.find(name) != m_SceneNameToIndex.end())
    {
        return false;
    }
    
    // Register the scene
    SceneInfo info;
    info.name = name;
    info.description = description;
    info.factory = factory;
    
    m_SceneRegistry.push_back(info);
    m_SceneNameToIndex[name] = m_SceneRegistry.size() - 1;
    
    return true;
}

bool TestSceneManager::SwitchToScene(const std::string& sceneName)
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    auto it = m_SceneNameToIndex.find(sceneName);
    if (it == m_SceneNameToIndex.end())
    {
        return false;
    }
    
    return SwitchToScene(it->second);
}

bool TestSceneManager::SwitchToScene(size_t sceneIndex)
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    // Validate index
    if (sceneIndex >= m_SceneRegistry.size())
    {
        return false;
    }
    
    // Don't switch if it's already the current scene
    if (m_CurrentSceneIndex == sceneIndex)
    {
        return true;
    }
    
    // Cleanup the current scene
    CleanupCurrentScene();
    
    // Create the new scene
    m_CurrentSceneIndex = sceneIndex;
    const auto& sceneInfo = m_SceneRegistry[sceneIndex];
    m_CurrentScene = sceneInfo.factory();
    
    // Initialize the new scene
    if (m_CurrentScene)
    {
        m_CurrentScene->Initialize();
        
        // Call scene change callbacks
        for (const auto& callbackPair : m_SceneChangeCallbacks)
        {
            callbackPair.second(sceneInfo.name);
        }
        
        return true;
    }
    
    // If we get here, scene creation failed
    m_CurrentSceneIndex = (size_t)-1;
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
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    return m_SceneRegistry.size();
}

std::vector<std::string> TestSceneManager::GetSceneNames() const
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    std::vector<std::string> names;
    names.reserve(m_SceneRegistry.size());
    
    for (const auto& scene : m_SceneRegistry)
    {
        names.push_back(scene.name);
    }
    
    return names;
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

size_t TestSceneManager::RegisterSceneChangeCallback(std::function<void(const std::string&)> callback)
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    
    size_t callbackId = m_NextCallbackId++;
    m_SceneChangeCallbacks[callbackId] = callback;
    
    return callbackId;
}

void TestSceneManager::UnregisterSceneChangeCallback(size_t callbackId)
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    m_SceneChangeCallbacks.erase(callbackId);
}

bool TestSceneManager::SceneExists(const std::string& sceneName) const
{
    std::lock_guard<std::mutex> lock(m_SceneMutex);
    return m_SceneNameToIndex.find(sceneName) != m_SceneNameToIndex.end();
}

void TestSceneManager::CleanupCurrentScene()
{
    // No need for lock, already locked by caller
    
    if (m_CurrentScene)
    {
        m_CurrentScene->Cleanup();
        m_CurrentScene.reset();
    }
    
    m_CurrentSceneIndex = (size_t)-1;
}

std::shared_ptr<TestSceneBase> CreateTestScene(TestSceneType type)
{
    switch (type)
    {
    case TestSceneType::PHYSX_HELLO_WORLD:
        return std::make_shared<PhysXHelloWorldScene>();
    default:
        return nullptr;
    } 
}