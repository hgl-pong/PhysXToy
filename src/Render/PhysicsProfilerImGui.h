#pragma once
#include "Renderer/Renderer.h"
#include "Physics/PhysicsCommon.h"
#include <string>
#include <vector>
#include <memory>
#include <imgui.h>

class PhysicsProfilerImGui : public GUIPanel
{
public:
    PhysicsProfilerImGui(IPhysicsProfiler* profiler);
    ~PhysicsProfilerImGui();

    void Render() override;
    void SetVisible(bool visible) override;
    bool IsVisible() const override;

private:
    void RenderPerformanceSummary();
    void RenderObjectStatistics();
    void RenderCollisionStatistics();
    void RenderMemoryUsage();
    void RenderDetailedEvents();
    void RenderFrameTimeGraph();
    void RenderExportOptions();
    
    void ExportToCSV(const std::string& filename);
    void ExportToJSON(const std::string& filename);
    void ExportToHTML(const std::string& filename);

    void PlotHistogram(const char* label, const std::vector<float>& values, float maxValue = 0.0f);

    IPhysicsProfiler* m_Profiler;
    bool m_Visible;
    bool m_ShowPerformance;
    bool m_ShowObjects;
    bool m_ShowCollisions;
    bool m_ShowMemory;
    bool m_ShowDetailedEvents;
    bool m_ShowFrameTimeGraph;
    bool m_ShowExportOptions;

    // Frame time history for graph rendering
    static constexpr int FRAME_HISTORY_COUNT = 120;
    std::vector<float> m_FrameTimeHistory;
    std::vector<float> m_PhysicsTimeHistory;
    int m_HistoryIndex;

    // UI state
    char m_ExportFilename[256];
    ImVec4 m_FrameTimeColor;
    ImVec4 m_PhysicsTimeColor;
    ImVec4 m_CollisionColor;
    ImVec4 m_SolverColor;
    float m_GraphHeight;

    // 缓存过滤选项
    bool m_SortByName = true;
    bool m_SortByTime = false;
    bool m_SortByCount = false;
    float m_TimeThreshold = 0.0f;
    char m_FilterBuffer[128] = "";
}; 