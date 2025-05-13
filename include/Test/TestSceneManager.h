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

    /**
     * Register a test scene factory function
     * @param name Scene name for display and reference
     * @param factory Factory function to create the scene
     * @param description Optional description of the scene
     * @return True if scene was registered successfully, false if name already exists
     */
    bool RegisterScene(const std::string& name, 
                      std::function<std::shared_ptr<TestSceneBase>()> factory,
                      const std::string& description = "");

    /**
     * Switch to a different scene by name
     * @param sceneName Name of the scene to switch to
     * @return True if scene switch was successful, false if scene not found
     */
    bool SwitchToScene(const std::string& sceneName);

    /**
     * Switch to a different scene by index
     * @param sceneIndex Index of the scene to switch to
     * @return True if scene switch was successful, false if index is invalid
     */
    bool SwitchToScene(size_t sceneIndex);
    
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
     * Get scene names of all registered scenes
     * @return Vector of scene names
     */
    std::vector<std::string> GetSceneNames() const;
    
    /**
     * Get scene description by name
     * @param sceneName Name of the scene
     * @return Description of the scene or empty string if not found
     */
    std::string GetSceneDescription() const;
    
    /**
     * Register a callback function to be called when scene changes
     * @param callback Function to call when scene changes
     * @return ID that can be used to unregister the callback
     */
    size_t RegisterSceneChangeCallback(std::function<void(const std::string&)> callback);
    
    /**
     * Unregister a scene change callback
     * @param callbackId ID of the callback to unregister
     */
    void UnregisterSceneChangeCallback(size_t callbackId);
    
    /**
     * Check if a scene with the given name exists
     * @param sceneName Name of the scene to check
     * @return True if scene exists, false otherwise
     */
    bool SceneExists(const std::string& sceneName) const;

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
    
    // Scene registry information
    struct SceneInfo
    {
        std::string name;
        std::string description;
        std::function<std::shared_ptr<TestSceneBase>()> factory;
    };
    
    std::vector<SceneInfo> m_SceneRegistry;
    std::unordered_map<std::string, size_t> m_SceneNameToIndex;
    
    // Current scene state
    std::shared_ptr<TestSceneBase> m_CurrentScene = nullptr;
    size_t m_CurrentSceneIndex = (size_t)-1;
    
    // Scene change callbacks
    std::unordered_map<size_t, std::function<void(const std::string&)>> m_SceneChangeCallbacks;
    size_t m_NextCallbackId = 0;
    
    // Thread safety
    mutable std::mutex m_SceneMutex;
};

extern std::shared_ptr<TestSceneBase> CreateTestScene(TestSceneType type);