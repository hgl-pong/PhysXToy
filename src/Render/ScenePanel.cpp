#include "ScenePanel.h"
#include <algorithm>
#include <imgui.h>

ScenePanel::ScenePanel()
    : m_SceneManager(TestSceneManager::GetInstance())
    , m_CurrentSceneIndex((size_t)-1)
    , m_Visible(true)
    , m_ConfirmSceneChange(false)
    , m_PendingSceneIndex((size_t)-1)
    , m_ShowAdvancedOptions(false)
{
    m_SearchFilter[0] = '\0';
    
    // 如果场景管理器中已有场景，设置当前索引为0
    if (m_SceneManager.GetSceneCount() > 0) {
        m_CurrentSceneIndex = 0;
    }
}

ScenePanel::~ScenePanel()
{
    // Clean up resources if needed
}

void ScenePanel::Render()
{
    if (!m_Visible)
        return;

    // Begin the scene panel window
    ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Scene Selection", &m_Visible))
    {
        ImGui::End();
        return;
    }

    // Show different sections of the panel
    RenderSceneSelection();
    
    auto sceneCount = m_SceneManager.GetSceneCount();
    if (m_CurrentSceneIndex != (size_t)-1 && m_CurrentSceneIndex < sceneCount)
    {
        ImGui::Separator();
        RenderSceneDetails();
        
        // 如果当前场景已加载，显示自定义参数
        auto currentScene = m_SceneManager.GetCurrentScene();
        if (currentScene && m_ShowAdvancedOptions)
        {
            ImGui::Separator();
            RenderCustomParameters();
        }
    }
    
    // Handle confirmation dialog
    if (m_ConfirmSceneChange)
    {
        ImGui::OpenPopup("Confirm Scene Change");
        m_ConfirmSceneChange = false;
    }
    
    // Show confirmation dialog
    if (ImGui::BeginPopupModal("Confirm Scene Change", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Changing scene will reset current simulation.");
        ImGui::Text("Any unsaved changes will be lost.");
        ImGui::Text("Are you sure you want to continue?");
        ImGui::Separator();
        
        if (ImGui::Button("Yes", ImVec2(120, 0)))
        {
            SetCurrentScene(m_PendingSceneIndex);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("No", ImVec2(120, 0)))
        {
            m_PendingSceneIndex = (size_t)-1;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    ImGui::End();
}

void ScenePanel::SetVisible(bool visible)
{
    m_Visible = visible;
}

bool ScenePanel::IsVisible() const
{
    return m_Visible;
}

size_t ScenePanel::GetCurrentSceneIndex() const
{
    return m_CurrentSceneIndex;
}

std::shared_ptr<TestSceneBase> ScenePanel::GetCurrentScene() const
{
    return m_SceneManager.GetCurrentScene();
}

void ScenePanel::SetCurrentScene(size_t index)
{
    if (index != (size_t)-1 && index != m_CurrentSceneIndex && index < m_SceneManager.GetSceneCount())
    {
        m_CurrentSceneIndex = index;
        
        // 使用SceneManager切换场景
        auto sceneNames = m_SceneManager.GetSceneNames();
        if (index < sceneNames.size()) {
            m_SceneManager.SwitchToScene(index);
        }
    }
}

void ScenePanel::SetCurrentScene(const std::string& sceneName)
{
    if (m_SceneManager.SceneExists(sceneName)) {
        m_SceneManager.SwitchToScene(sceneName);
        
        // 更新当前索引
        auto sceneNames = m_SceneManager.GetSceneNames();
        for (size_t i = 0; i < sceneNames.size(); i++) {
            if (sceneNames[i] == sceneName) {
                m_CurrentSceneIndex = i;
                break;
            }
        }
    }
}

void ScenePanel::ReloadCurrentScene()
{
    if (m_CurrentSceneIndex != (size_t)-1) {
        // 重新加载当前场景
        m_SceneManager.ResetCurrentScene();
    }
}

void ScenePanel::RenderSceneSelection()
{
    // Scene filtering/searching
    ImGui::Text("Filter:");
    ImGui::SameLine();
    ImGui::PushItemWidth(-1);
    ImGui::InputText("##SceneFilter", m_SearchFilter, sizeof(m_SearchFilter));
    ImGui::PopItemWidth();
    
    ImGui::Separator();
    ImGui::Text("Available Scenes:");
    
    // 获取所有场景名称
    auto sceneNames = m_SceneManager.GetSceneNames();
    
    // Display scene list with filtering
    if (ImGui::BeginListBox("##ScenesList", ImVec2(-1, 200)))
    {
        for (size_t i = 0; i < sceneNames.size(); i++)
        {
            const auto& sceneName = sceneNames[i];
            const auto& sceneDesc = m_SceneManager.GetSceneDescription(sceneName);
            
            // Apply filter if any
            if (m_SearchFilter[0] != '\0')
            {
                // Case-insensitive search
                std::string name = sceneName;
                std::string filter = m_SearchFilter;
                std::transform(name.begin(), name.end(), name.begin(), ::tolower);
                std::transform(filter.begin(), filter.end(), filter.begin(), ::tolower);
                
                if (name.find(filter) == std::string::npos)
                {
                    continue; // Skip this scene as it doesn't match the filter
                }
            }
            
            // Select the scene when clicked
            if (ImGui::Selectable(sceneName.c_str(), m_CurrentSceneIndex == i))
            {
                // Ask for confirmation before changing the scene
                m_PendingSceneIndex = i;
                m_ConfirmSceneChange = true;
            }
            
            // Show tooltip with description on hover
            if (ImGui::IsItemHovered() && !sceneDesc.empty())
            {
                ImGui::BeginTooltip();
                ImGui::TextUnformatted(sceneDesc.c_str());
                ImGui::EndTooltip();
            }
        }
        ImGui::EndListBox();
    }
    
    // Load button for the selected scene
    ImGui::BeginDisabled(m_CurrentSceneIndex == (size_t)-1);
    if (ImGui::Button("Load Selected Scene", ImVec2(-1, 0)))
    {
        // 直接调用SceneManager切换到当前选择的场景
        if (m_CurrentSceneIndex != (size_t)-1 && m_CurrentSceneIndex < sceneNames.size())
        {
            m_SceneManager.SwitchToScene(m_CurrentSceneIndex);
        }
    }
    ImGui::EndDisabled();
    
    // Reload current scene button
    ImGui::BeginDisabled(m_SceneManager.GetCurrentScene() == nullptr);
    if (ImGui::Button("Reload Current Scene", ImVec2(-1, 0)))
    {
        ReloadCurrentScene();
    }
    ImGui::EndDisabled();
}

void ScenePanel::RenderSceneDetails()
{
    auto sceneNames = m_SceneManager.GetSceneNames();
    if (m_CurrentSceneIndex == (size_t)-1 || m_CurrentSceneIndex >= sceneNames.size())
        return;
        
    const auto& sceneName = sceneNames[m_CurrentSceneIndex];
    const auto& sceneDesc = m_SceneManager.GetSceneDescription(sceneName);
    
    ImGui::Text("Selected Scene: %s", sceneName.c_str());
    
    if (!sceneDesc.empty())
    {
        ImGui::TextWrapped("Description: %s", sceneDesc.c_str());
    }
    
    // Show/hide advanced options
    ImGui::Checkbox("Show Advanced Options", &m_ShowAdvancedOptions);
}

void ScenePanel::RenderCustomParameters()
{
    auto currentScene = m_SceneManager.GetCurrentScene();
    if (!currentScene)
        return;
        
    ImGui::Text("Scene Parameters:");
    
    // Example parameters that might be adjusted for different scenes
    static float gravity = -9.8f;
    static int particleCount = 1000;
    static bool enableCollision = true;
    static float timeScale = 1.0f;
    
    ImGui::SliderFloat("Gravity", &gravity, -20.0f, 0.0f, "%.1f m/s²");
    ImGui::SliderInt("Particle Count", &particleCount, 100, 10000);
    ImGui::Checkbox("Enable Collision", &enableCollision);
    ImGui::SliderFloat("Time Scale", &timeScale, 0.1f, 2.0f, "%.2fx");
    
    // Apply button for parameter changes
    if (ImGui::Button("Apply Parameters", ImVec2(-1, 0)))
    {
        // 这里应该应用参数变更到物理模拟中
        // 具体实现可能需要基于应用程序的特定需求
    }
    
    // 添加场景控制按钮
    ImGui::Separator();
    ImGui::Text("Scene Controls:");
    
    if (ImGui::Button("Pause/Resume", ImVec2(120, 0)))
    {
        if (currentScene->IsPaused()) {
            m_SceneManager.ResumeCurrentScene();
        } else {
            m_SceneManager.PauseCurrentScene();
        }
    }
    
    ImGui::SameLine();
    if (ImGui::Button("Reset", ImVec2(120, 0)))
    {
        m_SceneManager.ResetCurrentScene();
    }
}
