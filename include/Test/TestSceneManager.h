#pragma once

#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include <mutex>

/**
 * TestSceneManager - Manages test scenes and provides safe scene switching
 * 
 * This class is responsible for:
 * 1. Registering test scenes
 * 2. Managing scene lifecycle (initialization, cleanup)
 * 3. Providing thread-safe scene switching
 * 4. Notifying listeners when scenes change
 */
class TestSceneManager
{
public:
    /**
     * Get the singleton instance of TestSceneManager
     * @return Reference to the TestSceneManager singleton
     */
    static TestSceneManager& GetInstance();


    bool SwitchToScene(TestSceneType type);
    
    /**
     * Get currently active scene
     * @return Pointer to current scene, nullptr if no scene is active
     */
    std::shared_ptr<TestSceneBase> GetCurrentScene() const;
    
    /**
     * Update the current scene
     * @param deltaTime Time elapsed since last update in seconds
     */
    void UpdateCurrentScene(float deltaTime);
    
    /**
     * Render the current scene
     */
    void RenderCurrentScene();
    
    /**
     * Reset the current scene
     */
    void ResetCurrentScene();
    
    /**
     * Pause the current scene
     */
    void PauseCurrentScene();
    
    /**
     * Resume the current scene
     */
    void ResumeCurrentScene();

    bool IsCurrentScenePause() const;
    
    /**
     * Get the number of registered scenes
     * @return Number of scenes
     */
    size_t GetSceneCount() const;

    /**
     * Get scene description by name
     * @param sceneName Name of the scene
     * @return Description of the scene or empty string if not found
     */
    std::string GetSceneDescription() const;
    
private:
    // Private constructor to enforce singleton pattern
    TestSceneManager() = default;
    ~TestSceneManager() = default;
    
    // Disable copy and move
    TestSceneManager(const TestSceneManager&) = delete;
    TestSceneManager& operator=(const TestSceneManager&) = delete;
    TestSceneManager(TestSceneManager&&) = delete;
    TestSceneManager& operator=(TestSceneManager&&) = delete;
    
    // Helper method to cleanup current scene and trigger callbacks
    void CleanupCurrentScene();
    
    // Current scene state
    std::shared_ptr<TestSceneBase> m_CurrentScene = nullptr;
    TestSceneType m_CurrentSceneType = TestSceneType::DEFAULT_SCENE;
    
    // Thread safety
    mutable std::mutex m_SceneMutex;
};

extern std::shared_ptr<TestSceneBase> CreateTestScene(TestSceneType type);