#pragma once
#include "Renderer/Renderer.h"
#include "Physics/PhysicsCommon.h"
#include "Test/TestSceneManager.h"
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <imgui.h>

// Forward declarations
class TestSceneBase;

/**
 * ScenePanel - ImGui panel for managing scene selection and switching
 * Allows users to select from different predefined scenes and customize scene parameters
 * Uses TestSceneManager to manage scene lifecycle
 */
class ScenePanel : public GUIPanel
{
public:
    // Constructor
    ScenePanel();
    ~ScenePanel();

    // GUIPanel interface implementation
    void Render() override;
    void SetVisible(bool visible) override;
    bool IsVisible() const override;
    
    // Get currently selected scene
    size_t GetCurrentSceneIndex() const;
    
    // Get currently active scene
    std::shared_ptr<TestSceneBase> GetCurrentScene() const;
    
    // Set current scene by index
    void SetCurrentScene(size_t index);
    
    // Set current scene by name
    void SetCurrentScene(const std::string& sceneName);
    
    // Reload current scene
    void ReloadCurrentScene();

private:
    // Render different sections of the panel
    void RenderSceneSelection();
    void RenderSceneDetails();
    void RenderCustomParameters();
    
    // Reference to the scene manager
    TestSceneManager& m_SceneManager;
    
    size_t m_CurrentSceneIndex;                     // Currently selected scene index
    bool m_Visible;                                 // Panel visibility flag
    
    // UI state variables
    bool m_ConfirmSceneChange;                      // Whether to show confirmation dialog
    size_t m_PendingSceneIndex;                     // Scene waiting for confirmation
    char m_SearchFilter[128];                       // Scene search/filter text
    bool m_ShowAdvancedOptions;                     // Whether to show advanced options
};
