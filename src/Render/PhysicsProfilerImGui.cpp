#include "PhysicsProfilerImGui.h"
#include <imgui.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <float.h>

PhysicsProfilerImGui::PhysicsProfilerImGui(IPhysicsProfiler* profiler)
    : m_Profiler(profiler)
    , m_Visible(true)
    , m_ShowPerformance(true)
    , m_ShowObjects(true)
    , m_ShowCollisions(true)
    , m_ShowMemory(true)
    , m_ShowDetailedEvents(false)
    , m_ShowFrameTimeGraph(true)
    , m_ShowExportOptions(false)
    , m_HistoryIndex(0)
    , m_GraphHeight(80.0f)
{
    // Initialize frame history
    m_FrameTimeHistory.resize(FRAME_HISTORY_COUNT, 0.0f);
    m_PhysicsTimeHistory.resize(FRAME_HISTORY_COUNT, 0.0f);
    
    // Set colors
    m_FrameTimeColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    m_PhysicsTimeColor = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    m_CollisionColor = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
    m_SolverColor = ImVec4(0.0f, 1.0f, 1.0f, 1.0f);
    
    // Initialize export filename with default
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "PhysicsStats_" 
        << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string defaultName = oss.str();
    strncpy(m_ExportFilename, defaultName.c_str(), sizeof(m_ExportFilename) - 1);
    m_ExportFilename[sizeof(m_ExportFilename) - 1] = '\0';
}

PhysicsProfilerImGui::~PhysicsProfilerImGui()
{
    // Nothing to clean up
}

void PhysicsProfilerImGui::SetVisible(bool visible)
{
    m_Visible = visible;
}

bool PhysicsProfilerImGui::IsVisible() const
{
    return m_Visible;
}

void PhysicsProfilerImGui::Render()
{
    if (!m_Visible || !m_Profiler)
        return;
        
    // Update frame history with latest data
    const auto& latest = m_Profiler->GetLatestFrameStats();
    
    // Convert microseconds to milliseconds for display
    m_FrameTimeHistory[m_HistoryIndex] = latest.frameDuration * 0.001f;
    m_PhysicsTimeHistory[m_HistoryIndex] = latest.physicsStepTime * 0.001f;
    
    m_HistoryIndex = (m_HistoryIndex + 1) % FRAME_HISTORY_COUNT;
    
    // Begin ImGui window
    ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Physics Engine Statistics", &m_Visible))
    {
        ImGui::End();
        return;
    }

    // Add main tabs
    if (ImGui::BeginTabBar("PhysicsProfilerTabs"))
    {
        if (ImGui::BeginTabItem("Performance"))
        {
            RenderPerformanceSummary();
            RenderFrameTimeGraph();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem("Objects"))
        {
            RenderObjectStatistics();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem("Collisions"))
        {
            RenderCollisionStatistics();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem("Memory"))
        {
            RenderMemoryUsage();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem("Events"))
        {
            RenderDetailedEvents();
            ImGui::EndTabItem();
        }
        
        if (ImGui::BeginTabItem("Export"))
        {
            RenderExportOptions();
            ImGui::EndTabItem();
        }
        
        ImGui::EndTabBar();
    }
    
    // Controls at the bottom
    ImGui::Separator();
    if (ImGui::Button("Reset Statistics"))
    {
        m_Profiler->ResetStatistics();
        std::fill(m_FrameTimeHistory.begin(), m_FrameTimeHistory.end(), 0.0f);
        std::fill(m_PhysicsTimeHistory.begin(), m_PhysicsTimeHistory.end(), 0.0f);
        m_HistoryIndex = 0;
    }
    
    ImGui::SameLine();
    
    // Toggle for verbose output
    bool verboseOutput = false;
    if (ImGui::Checkbox("Verbose Output", &verboseOutput))
    {
        m_Profiler->SetVerboseOutput(verboseOutput);
    }
    
    ImGui::End();
}

void PhysicsProfilerImGui::RenderPerformanceSummary()
{
    if (!m_Profiler)
        return;
        
    const auto& latest = m_Profiler->GetLatestFrameStats();
    
    ImGui::Text("Performance Summary:");
    ImGui::Separator();
    
    // Convert microseconds to milliseconds for display
    const float msScale = 0.001f;
    
    ImGui::Columns(2);
    ImGui::Text("Frame Time:"); ImGui::NextColumn();
    ImGui::Text("%.2f ms", latest.frameDuration * msScale); ImGui::NextColumn();
    
    ImGui::Text("Physics Step:"); ImGui::NextColumn();
    ImGui::Text("%.2f ms (%.1f%%)", latest.physicsStepTime * msScale,
                m_Profiler->GetPhysicsTimePercentage()); ImGui::NextColumn();
    
    ImGui::Text("Collision Detection:"); ImGui::NextColumn();
    ImGui::Text("%.2f ms", latest.collisionDetectionTime * msScale); ImGui::NextColumn();
    
    ImGui::Text("Solver:"); ImGui::NextColumn();
    ImGui::Text("%.2f ms", latest.solverTime * msScale); ImGui::NextColumn();
    
    ImGui::Text("Integration:"); ImGui::NextColumn();
    ImGui::Text("%.2f ms", latest.integrateTime * msScale); ImGui::NextColumn();
    
    ImGui::Text("Average Frame Time:"); ImGui::NextColumn();
    ImGui::Text("%.2f ms", m_Profiler->GetAverageFrameTime() * msScale); ImGui::NextColumn();
    
    ImGui::Text("Peak Frame Time:"); ImGui::NextColumn();
    ImGui::Text("%.2f ms", m_Profiler->GetPeakFrameTime() * msScale); ImGui::NextColumn();
    
    ImGui::Columns(1);
}

void PhysicsProfilerImGui::RenderObjectStatistics()
{
    if (!m_Profiler)
        return;
        
    const auto& latest = m_Profiler->GetLatestFrameStats();
    
    ImGui::Text("Object Statistics:");
    ImGui::Separator();
    
    ImGui::Columns(2);
    ImGui::Text("Total Objects:"); ImGui::NextColumn();
    ImGui::Text("%u", latest.activeObjects); ImGui::NextColumn();
    
    ImGui::Text("Dynamic Objects:"); ImGui::NextColumn();
    ImGui::Text("%u", latest.activeDynamicObjects); ImGui::NextColumn();
    
    ImGui::Text("Static Objects:"); ImGui::NextColumn();
    ImGui::Text("%u", latest.activeStaticObjects); ImGui::NextColumn();
    
    ImGui::Text("Soft Bodies:"); ImGui::NextColumn();
    ImGui::Text("%u", latest.activeSoftBodies); ImGui::NextColumn();
    
    ImGui::Text("Joints:"); ImGui::NextColumn();
    ImGui::Text("%u", latest.activeJoints); ImGui::NextColumn();
    
    ImGui::Columns(1);
    
    // Display historical object count chart
    if (ImGui::CollapsingHeader("Object Count History", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Placeholder for a graph of object counts over time
        ImGui::Text("Object count history graph would be displayed here");
    }
}

void PhysicsProfilerImGui::RenderCollisionStatistics()
{
    if (!m_Profiler)
        return;
        
    const auto& latest = m_Profiler->GetLatestFrameStats();
    
    ImGui::Text("Collision Statistics:");
    ImGui::Separator();
    
    ImGui::Columns(2);
    ImGui::Text("Contact Points:"); ImGui::NextColumn();
    ImGui::Text("%u", latest.contactPoints); ImGui::NextColumn();
    
    ImGui::Text("Collision Pairs:"); ImGui::NextColumn();
    ImGui::Text("%u", latest.collisionPairs); ImGui::NextColumn();
    
    ImGui::Columns(1);
    
    // Display historical collision data chart
    if (ImGui::CollapsingHeader("Collision History", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Placeholder for a graph of collision data over time
        ImGui::Text("Collision history graph would be displayed here");
    }
}

void PhysicsProfilerImGui::RenderMemoryUsage()
{
    if (!m_Profiler)
        return;
        
    const auto& latest = m_Profiler->GetLatestFrameStats();
    
    ImGui::Text("Memory Usage:");
    ImGui::Separator();
    
    // Convert bytes to more readable units
    float memoryMB = latest.memoryUsage / (1024.0f * 1024.0f);
    
    ImGui::Columns(2);
    ImGui::Text("Total Memory:"); ImGui::NextColumn();
    ImGui::Text("%.2f MB", memoryMB); ImGui::NextColumn();
    
    // Additional memory metrics would go here
    
    ImGui::Columns(1);
    
    // Display historical memory usage chart
    if (ImGui::CollapsingHeader("Memory History", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Placeholder for a graph of memory usage over time
        ImGui::Text("Memory usage history graph would be displayed here");
    }
}

void PhysicsProfilerImGui::RenderDetailedEvents()
{
    if (!m_Profiler)
        return;
        
    ImGui::Text("Event Timing Details:");
    ImGui::Separator();
    
    // Get event statistics from profiler
    const auto& eventStats = m_Profiler->GetEventStats();
    
    if (eventStats.empty())
    {
        ImGui::Text("No events recorded yet.");
        return;
    }
    
    // Event timing table
    if (ImGui::BeginTable("EventTimings", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("Event Name");
        ImGui::TableSetupColumn("Count");
        ImGui::TableSetupColumn("Avg Time (ms)");
        ImGui::TableSetupColumn("Min Time (ms)");
        ImGui::TableSetupColumn("Max Time (ms)");
        ImGui::TableHeadersRow();
        
        for (const auto& pair : eventStats)
        {
            const std::string& name = pair.first;
            const ProfileEventStats& stats = pair.second;
            double avgTime = stats.count > 0 ? 
                static_cast<double>(stats.totalTime) / stats.count * 0.001 : 0.0;
            double minTime = stats.minTime == UINT64_MAX ? 0.0 : static_cast<double>(stats.minTime) * 0.001;
            double maxTime = static_cast<double>(stats.maxTime) * 0.001;
            
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%s", name.c_str());
            ImGui::TableNextColumn(); ImGui::Text("%u", stats.count);
            ImGui::TableNextColumn(); ImGui::Text("%.3f", avgTime);
            ImGui::TableNextColumn(); ImGui::Text("%.3f", minTime);
            ImGui::TableNextColumn(); ImGui::Text("%.3f", maxTime);
        }
        
        ImGui::EndTable();
    }
}

void PhysicsProfilerImGui::RenderFrameTimeGraph()
{
    if (!m_Profiler)
        return;
        
    ImGui::Text("Frame Time History:");
    ImGui::Separator();
    
    // Graph controls
    ImGui::SliderFloat("Graph Height", &m_GraphHeight, 40.0f, 200.0f);
    
    // Calculate min and max values for better scaling
    float minValue = FLT_MAX;
    float maxValue = -FLT_MAX;
    
    for (float value : m_FrameTimeHistory)
    {
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }
    
    for (float value : m_PhysicsTimeHistory)
    {
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }
    
    // Ensure we have a reasonable range
    if (minValue == FLT_MAX) minValue = 0.0f;
    if (maxValue == -FLT_MAX) maxValue = 16.0f;  // Default to 60fps (16.6ms)
    
    // Add some padding to the range
    float range = maxValue - minValue;
    minValue = std::max(0.0f, minValue - range * 0.1f);
    maxValue = maxValue + range * 0.1f;
    
    // Convert circular buffer to linear for plotting
    std::vector<float> sortedFrameTimes(FRAME_HISTORY_COUNT);
    std::vector<float> sortedPhysicsTimes(FRAME_HISTORY_COUNT);
    
    for (int i = 0; i < FRAME_HISTORY_COUNT; i++)
    {
        int idx = (m_HistoryIndex + i) % FRAME_HISTORY_COUNT;
        sortedFrameTimes[i] = m_FrameTimeHistory[idx];
        sortedPhysicsTimes[i] = m_PhysicsTimeHistory[idx];
    }
    
    // Create the plot
    ImGui::PlotLines("##FrameTime", sortedFrameTimes.data(), sortedFrameTimes.size(), 
                    0, "Frame Time (ms)", minValue, maxValue, ImVec2(ImGui::GetContentRegionAvail().x, m_GraphHeight));
                    
    // Display legend and statistics
    ImGui::Text("Frame Time: %.2f ms (min: %.2f, max: %.2f)", 
                sortedFrameTimes.back(), minValue, maxValue);
                
    ImGui::PlotLines("##PhysicsTime", sortedPhysicsTimes.data(), sortedPhysicsTimes.size(), 
                    0, "Physics Time (ms)", minValue, maxValue, ImVec2(ImGui::GetContentRegionAvail().x, m_GraphHeight));
                    
    ImGui::Text("Physics Time: %.2f ms", sortedPhysicsTimes.back());
}

void PhysicsProfilerImGui::RenderExportOptions()
{
    ImGui::Text("Export Statistics:");
    ImGui::Separator();
    
    ImGui::InputText("Filename", m_ExportFilename, sizeof(m_ExportFilename));
    
    if (ImGui::Button("Export to CSV"))
    {
        std::string filename = std::string(m_ExportFilename) + ".csv";
        ExportToCSV(filename);
        ImGui::OpenPopup("Export Complete");
    }
    
    ImGui::SameLine();
    
    if (ImGui::Button("Export to JSON"))
    {
        std::string filename = std::string(m_ExportFilename) + ".json";
        ExportToJSON(filename);
        ImGui::OpenPopup("Export Complete");
    }
    
    ImGui::SameLine();
    
    if (ImGui::Button("Export to HTML"))
    {
        std::string filename = std::string(m_ExportFilename) + ".html";
        ExportToHTML(filename);
        ImGui::OpenPopup("Export Complete");
    }
    
    if (ImGui::BeginPopupModal("Export Complete", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Statistics exported successfully!");
        if (ImGui::Button("OK", ImVec2(120, 0)))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void PhysicsProfilerImGui::ExportToCSV(const std::string& filename)
{
    // Call the interface method to export
    m_Profiler->ExportStatisticsToCSV(filename);
}

void PhysicsProfilerImGui::ExportToJSON(const std::string& filename)
{
    // Call the interface method to export
    m_Profiler->ExportStatisticsToJSON(filename);
}

void PhysicsProfilerImGui::ExportToHTML(const std::string& filename)
{
    // Call the interface method to export
    m_Profiler->ExportStatisticsToHTML(filename);
} 