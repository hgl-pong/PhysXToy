#pragma once
#include "Physics/PhysicsCommon.h"
#include "PhysicsProfiler.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <vector>

class PhysicsProfiler;
class PhysicsStatistic;

class PhysicsProfilerChart
{
public:

    PhysicsProfilerChart(PhysicsProfiler* profiler) : m_Profiler(profiler) {}

    bool exportData(const std::string& filepath, ProfileChartExportFormat format = ProfileChartExportFormat::CSV)
    {
        if (!m_Profiler)
            return false;

        std::ofstream file(filepath);
        if (!file.is_open())
            return false;

        bool result = false;

        switch (format) 
        {
            case ProfileChartExportFormat::CSV:
                result = exportToCSV(file);
                break;
            case ProfileChartExportFormat::JSON:
                result = exportToJSON(file);
                break;
            case ProfileChartExportFormat::HTML:
                result = exportToHTML(file);
                break;
        }

        file.close();
        return result;
    }
    
    // Export PhysicsStatistic data
    bool exportStatisticsData(const std::string& filepath, ProfileChartExportFormat format = ProfileChartExportFormat::HTML)
    {
        if (!m_Profiler)
            return false;

        std::ofstream file(filepath);
        if (!file.is_open())
            return false;

        bool result = false;

        switch (format) 
        {
            case ProfileChartExportFormat::CSV:
                result = exportStatisticsToCSV(file);
                break;
            case ProfileChartExportFormat::JSON:
                result = exportStatisticsToJSON(file);
                break;
            case ProfileChartExportFormat::HTML:
                result = exportStatisticsToHTML(file);
                break;
        }

        file.close();
        return result;
    }

private:
    bool exportToCSV(std::ofstream& file)
    {
        file << "Event,Count,TotalTime(us),AvgTime(us),MinTime(us),MaxTime(us)\n";
        
        const auto& stats = m_Profiler->getEventStats();
        
        for (const auto& pair : stats)
        {
            const std::string& name = pair.first;
            const ProfileEventStats& stat = pair.second;
            double avgTime = stat.count > 0 ? static_cast<double>(stat.totalTime) / stat.count : 0.0;
            
            file << name << "," 
                 << stat.count << "," 
                 << stat.totalTime << "," 
                 << avgTime << "," 
                 << stat.minTime << "," 
                 << stat.maxTime << "\n";
        }
        
        return true;
    }
    
    // Export PhysicsStatistic data to CSV format
    bool exportStatisticsToCSV(std::ofstream& file)
    {
        const auto& statistic = m_Profiler->GetStatistic();
        const auto& frameHistory = statistic.GetFrameHistory();
        
        // Write headers
        file << "Frame,FrameStartTime,FrameEndTime,FrameDuration,PhysicsStepTime,CollisionDetectionTime,SolverTime,IntegrateTime,"
             << "ActiveObjects,ActiveDynamicObjects,ActiveStaticObjects,ActiveSoftBodies,ActiveJoints,"
             << "ContactPoints,CollisionPairs,MemoryUsage,DiscreteContactPairs,CacheHitPairs,ContactPairsWithContacts,"
             << "NewPairs,LostPairs,NewTouches,LostTouches,Partitions,ActiveConstraints,ActiveDynamicBodies,"
             << "ActiveKinematicBodies,StaticBodies,DynamicBodies,KinematicBodies,Aggregates,Articulations,"
             << "AxisSolverConstraints,CompressedContactSize,RequiredContactConstraintMemory,PeakConstraintMemory,"
             << "BroadphaseAdds,BroadphaseRemoves,GPUMemParticles,GPUMemSoftBodies,GPUMemFEMCloths,"
             << "GPUMemHairSystems,GPUMemHeap,GPUMemHeapBroadPhase,GPUMemHeapNarrowPhase,GPUMemHeapSolver,"
             << "GPUMemHeapArticulation,GPUMemHeapSimulation,TotalGPUMemory\n";
        
        // Write data for each frame
        for (size_t i = 0; i < frameHistory.size(); i++)
        {
            const auto& frame = frameHistory[i];
            file << i << "," 
                 << frame.frameStartTime << "," 
                 << frame.frameEndTime << "," 
                 << frame.frameDuration << "," 
                 << frame.physicsStepTime << "," 
                 << frame.collisionDetectionTime << "," 
                 << frame.solverTime << "," 
                 << frame.integrateTime << "," 
                 << frame.activeObjects << "," 
                 << frame.activeDynamicObjects << "," 
                 << frame.activeStaticObjects << "," 
                 << frame.activeSoftBodies << "," 
                 << frame.activeJoints << "," 
                 << frame.contactPoints << "," 
                 << frame.collisionPairs << "," 
                 << frame.memoryUsage << ","
                 << frame.discreteContactPairs << ","
                 << frame.cacheHitPairs << ","
                 << frame.contactPairsWithContacts << ","
                 << frame.newPairs << ","
                 << frame.lostPairs << ","
                 << frame.newTouches << ","
                 << frame.lostTouches << ","
                 << frame.partitions << ","
                 << frame.activeConstraints << ","
                 << frame.activeDynamicBodies << ","
                 << frame.activeKinematicBodies << ","
                 << frame.staticBodies << ","
                 << frame.dynamicBodies << ","
                 << frame.kinematicBodies << ","
                 << frame.aggregates << ","
                 << frame.articulations << ","
                 << frame.axisSolverConstraints << ","
                 << frame.compressedContactSize << ","
                 << frame.requiredContactConstraintMemory << ","
                 << frame.peakConstraintMemory << ","
                 << frame.broadphaseAdds << ","
                 << frame.broadphaseRemoves << ","
                 << frame.gpuMemParticles << ","
                 << frame.gpuMemSoftBodies << ","
                 << frame.gpuMemFEMCloths << ","
                 << frame.gpuMemHairSystems << ","
                 << frame.gpuMemHeap << ","
                 << frame.gpuMemHeapBroadPhase << ","
                 << frame.gpuMemHeapNarrowPhase << ","
                 << frame.gpuMemHeapSolver << ","
                 << frame.gpuMemHeapArticulation << ","
                 << frame.gpuMemHeapSimulation << ","
                 << frame.totalGPUMemory << "\n";
        }
        
        // Write summary statistics
        file << "\nSummary Statistics\n";
        file << "Average Frame Time (us)," << statistic.GetAverageFrameTime() << "\n";
        file << "Average Physics Step Time (us)," << statistic.GetAveragePhysicsStepTime() << "\n";
        file << "Peak Frame Time (us)," << statistic.GetPeakFrameTime() << "\n";
        file << "Physics Time Percentage (%)," << statistic.GetPhysicsTimePercentage() << "\n";
        
        return true;
    }

    bool exportToJSON(std::ofstream& file)
    {
        const auto& stats = m_Profiler->getEventStats();
        const auto& events = m_Profiler->getCompletedEvents();
        const auto& dataRecords = m_Profiler->getDataRecords();
        
        file << "{\n  \"profilerData\": {\n";
        
        file << "    \"eventStats\": [\n";
        bool first = true;
        for (const auto& pair : stats)
        {
            if (!first) file << ",\n";
            first = false;
            
            const std::string& name = pair.first;
            const ProfileEventStats& stat = pair.second;
            double avgTime = stat.count > 0 ? static_cast<double>(stat.totalTime) / stat.count : 0.0;
            
            file << "      {\n"
                 << "        \"event\": \"" << name << "\",\n"
                 << "        \"count\": " << stat.count << ",\n"
                 << "        \"totalTime\": " << stat.totalTime << ",\n"
                 << "        \"avgTime\": " << avgTime << ",\n"
                 << "        \"minTime\": " << stat.minTime << ",\n"
                 << "        \"maxTime\": " << stat.maxTime << "\n"
                 << "      }";
        }
        file << "\n    ],\n";
        
        file << "    \"events\": [\n";
        first = true;
        for (const auto& event : events)
        {
            if (!first) file << ",\n";
            first = false;
            
            file << "      {\n"
                 << "        \"name\": \"" << event.name << "\",\n"
                 << "        \"contextId\": " << event.contextId << ",\n"
                 << "        \"startTime\": " << event.startTime << ",\n"
                 << "        \"endTime\": " << event.endTime << ",\n"
                 << "        \"duration\": " << event.duration << "\n"
                 << "      }";
        }
        file << "\n    ],\n";
        
        file << "    \"dataRecords\": [\n";
        first = true;
        for (const auto& record : dataRecords)
        {
            if (!first) file << ",\n";
            first = false;
            
            file << "      {\n"
                 << "        \"name\": \"" << record.name << "\",\n"
                 << "        \"contextId\": " << record.contextId << ",\n";
                 
            if (record.isFloat)
                file << "        \"value\": " << record.floatValue;
            else
                file << "        \"value\": " << record.intValue;
                
            file << ",\n        \"isFloat\": " << (record.isFloat ? "true" : "false") << "\n"
                 << "      }";
        }
        file << "\n    ]\n";
        
        file << "  }\n}\n";
        
        return true;
    }
    
    // Export PhysicsStatistic data to JSON format
    bool exportStatisticsToJSON(std::ofstream& file)
    {
        const auto& statistic = m_Profiler->GetStatistic();
        const auto& frameHistory = statistic.GetFrameHistory();
        
        file << "{\n";
        file << "  \"statisticsData\": {\n";
        
        // Export frame history
        file << "    \"frameHistory\": [\n";
        for (size_t i = 0; i < frameHistory.size(); i++)
        {
            if (i > 0) file << ",\n";
            
            const auto& frame = frameHistory[i];
            file << "      {\n"
                 << "        \"frameIndex\": " << i << ",\n"
                 << "        \"frameStartTime\": " << frame.frameStartTime << ",\n"
                 << "        \"frameEndTime\": " << frame.frameEndTime << ",\n"
                 << "        \"frameDuration\": " << frame.frameDuration << ",\n"
                 << "        \"physicsStepTime\": " << frame.physicsStepTime << ",\n"
                 << "        \"collisionDetectionTime\": " << frame.collisionDetectionTime << ",\n"
                 << "        \"solverTime\": " << frame.solverTime << ",\n"
                 << "        \"integrateTime\": " << frame.integrateTime << ",\n"
                 << "        \"activeObjects\": " << frame.activeObjects << ",\n"
                 << "        \"activeDynamicObjects\": " << frame.activeDynamicObjects << ",\n"
                 << "        \"activeStaticObjects\": " << frame.activeStaticObjects << ",\n"
                 << "        \"activeSoftBodies\": " << frame.activeSoftBodies << ",\n"
                 << "        \"activeJoints\": " << frame.activeJoints << ",\n"
                 << "        \"contactPoints\": " << frame.contactPoints << ",\n"
                 << "        \"collisionPairs\": " << frame.collisionPairs << ",\n"
                 << "        \"memoryUsage\": " << frame.memoryUsage << ",\n"
                 << "        \"discreteContactPairs\": " << frame.discreteContactPairs << ",\n"
                 << "        \"cacheHitPairs\": " << frame.cacheHitPairs << ",\n"
                 << "        \"contactPairsWithContacts\": " << frame.contactPairsWithContacts << ",\n"
                 << "        \"newPairs\": " << frame.newPairs << ",\n"
                 << "        \"lostPairs\": " << frame.lostPairs << ",\n"
                 << "        \"newTouches\": " << frame.newTouches << ",\n"
                 << "        \"lostTouches\": " << frame.lostTouches << ",\n"
                 << "        \"partitions\": " << frame.partitions << ",\n"
                 << "        \"activeConstraints\": " << frame.activeConstraints << ",\n"
                 << "        \"activeDynamicBodies\": " << frame.activeDynamicBodies << ",\n"
                 << "        \"activeKinematicBodies\": " << frame.activeKinematicBodies << ",\n"
                 << "        \"staticBodies\": " << frame.staticBodies << ",\n"
                 << "        \"dynamicBodies\": " << frame.dynamicBodies << ",\n"
                 << "        \"kinematicBodies\": " << frame.kinematicBodies << ",\n"
                 << "        \"aggregates\": " << frame.aggregates << ",\n"
                 << "        \"articulations\": " << frame.articulations << ",\n"
                 << "        \"axisSolverConstraints\": " << frame.axisSolverConstraints << ",\n"
                 << "        \"compressedContactSize\": " << frame.compressedContactSize << ",\n"
                 << "        \"requiredContactConstraintMemory\": " << frame.requiredContactConstraintMemory << ",\n"
                 << "        \"peakConstraintMemory\": " << frame.peakConstraintMemory << ",\n"
                 << "        \"broadphaseAdds\": " << frame.broadphaseAdds << ",\n"
                 << "        \"broadphaseRemoves\": " << frame.broadphaseRemoves << ",\n"
                 << "        \"gpuMemParticles\": " << frame.gpuMemParticles << ",\n"
                 << "        \"gpuMemSoftBodies\": " << frame.gpuMemSoftBodies << ",\n"
                 << "        \"gpuMemFEMCloths\": " << frame.gpuMemFEMCloths << ",\n"
                 << "        \"gpuMemHairSystems\": " << frame.gpuMemHairSystems << ",\n"
                 << "        \"gpuMemHeap\": " << frame.gpuMemHeap << ",\n"
                 << "        \"gpuMemHeapBroadPhase\": " << frame.gpuMemHeapBroadPhase << ",\n"
                 << "        \"gpuMemHeapNarrowPhase\": " << frame.gpuMemHeapNarrowPhase << ",\n"
                 << "        \"gpuMemHeapSolver\": " << frame.gpuMemHeapSolver << ",\n"
                 << "        \"gpuMemHeapArticulation\": " << frame.gpuMemHeapArticulation << ",\n"
                 << "        \"gpuMemHeapSimulation\": " << frame.gpuMemHeapSimulation << ",\n"
                 << "        \"totalGPUMemory\": " << frame.totalGPUMemory << "\n"
                 << "      }";
        }
        file << "\n    ],\n";
        
        // Export summary statistics
        file << "    \"summary\": {\n"
             << "      \"averageFrameTime\": " << statistic.GetAverageFrameTime() << ",\n"
             << "      \"averagePhysicsStepTime\": " << statistic.GetAveragePhysicsStepTime() << ",\n"
             << "      \"peakFrameTime\": " << statistic.GetPeakFrameTime() << ",\n"
             << "      \"physicsTimePercentage\": " << statistic.GetPhysicsTimePercentage() << "\n"
             << "    }\n";
        
        file << "  }\n";
        file << "}\n";
        
        return true;
    }

    bool exportToHTML(std::ofstream& file)
    {
        const auto& stats = m_Profiler->getEventStats();
        
        std::stringstream labels, avgData, minData, maxData;
        for (const auto& pair : stats)
        {
            const std::string& name = pair.first;
            const ProfileEventStats& stat = pair.second;
            double avgTime = stat.count > 0 ? static_cast<double>(stat.totalTime) / stat.count : 0.0;
            
            labels << (labels.str().empty() ? "" : ", ") << "'" << name << "'";
            avgData << (avgData.str().empty() ? "" : ", ") << avgTime;
            minData << (minData.str().empty() ? "" : ", ") << stat.minTime;
            maxData << (maxData.str().empty() ? "" : ", ") << stat.maxTime;
        }
        
        file << "<!DOCTYPE html>\n"
             << "<html>\n"
             << "<head>\n"
             << "  <title>Physics Profiler Chart</title>\n"
             << "  <script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>\n"
             << "  <style>\n"
             << "    .chart-container { width: 800px; height: 400px; margin: 20px auto; }\n"
             << "    table { width: 800px; margin: 20px auto; border-collapse: collapse; }\n"
             << "    th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }\n"
             << "    th { background-color: #f2f2f2; }\n"
             << "    tr:nth-child(even) { background-color: #f9f9f9; }\n"
             << "  </style>\n"
             << "</head>\n"
             << "<body>\n"
             << "  <div class=\"chart-container\">\n"
             << "    <canvas id=\"profileChart\"></canvas>\n"
             << "  </div>\n"
             << "  <table>\n"
             << "    <tr>\n"
             << "      <th>Event</th>\n"
             << "      <th>Count</th>\n"
             << "      <th>Total Time (us)</th>\n"
             << "      <th>Avg Time (us)</th>\n"
             << "      <th>Min Time (us)</th>\n"
             << "      <th>Max Time (us)</th>\n"
             << "    </tr>\n";
             
        for (const auto& pair : stats)
        {
            const std::string& name = pair.first;
            const ProfileEventStats& stat = pair.second;
            double avgTime = stat.count > 0 ? static_cast<double>(stat.totalTime) / stat.count : 0.0;
            
            file << "    <tr>\n"
                 << "      <td>" << name << "</td>\n"
                 << "      <td>" << stat.count << "</td>\n"
                 << "      <td>" << stat.totalTime << "</td>\n"
                 << "      <td>" << avgTime << "</td>\n"
                 << "      <td>" << stat.minTime << "</td>\n"
                 << "      <td>" << stat.maxTime << "</td>\n"
                 << "    </tr>\n";
        }
             
        file << "  </table>\n"
             << "  <script>\n"
             << "    const ctx = document.getElementById('profileChart');\n"
             << "    const data = {\n"
             << "      labels: [" << labels.str() << "],\n"
             << "      datasets: [{\n"
             << "        label: 'Average Time (us)',\n"
             << "        data: [" << avgData.str() << "],\n"
             << "        backgroundColor: 'rgba(54, 162, 235, 0.5)',\n"
             << "        borderColor: 'rgb(54, 162, 235)',\n"
             << "        borderWidth: 1\n"
             << "      }, {\n"
             << "        label: 'Min Time (us)',\n"
             << "        data: [" << minData.str() << "],\n"
             << "        backgroundColor: 'rgba(75, 192, 192, 0.5)',\n"
             << "        borderColor: 'rgb(75, 192, 192)',\n"
             << "        borderWidth: 1\n"
             << "      }, {\n"
             << "        label: 'Max Time (us)',\n"
             << "        data: [" << maxData.str() << "],\n"
             << "        backgroundColor: 'rgba(255, 99, 132, 0.5)',\n"
             << "        borderColor: 'rgb(255, 99, 132)',\n"
             << "        borderWidth: 1\n"
             << "      }]\n"
             << "    };\n"
             << "    new Chart(ctx, {\n"
             << "      type: 'bar',\n"
             << "      data: data,\n"
             << "      options: {\n"
             << "        responsive: true,\n"
             << "        plugins: {\n"
             << "          title: {\n"
             << "            display: true,\n"
             << "            text: 'Physics Profiler Performance Data'\n"
             << "          },\n"
             << "        },\n"
             << "        scales: {\n"
             << "          y: {\n"
             << "            beginAtZero: true\n"
             << "          }\n"
             << "        }\n"
             << "      }\n"
             << "    });\n"
             << "  </script>\n"
             << "</body>\n"
             << "</html>\n";
        
        return true;
    }

    // Export PhysicsStatistic data to HTML format with multiple charts
    bool exportStatisticsToHTML(std::ofstream& file)
    {
        const auto& statistic = m_Profiler->GetStatistic();
        const auto& frameHistory = statistic.GetFrameHistory();
        
        // Prepare data series
        std::stringstream frameIndices, frameDurations, physicsStepTimes, collisionTimes;
        std::stringstream solverTimes, integrateTimes, objectCounts, dynamicObjects, staticObjects;
        std::stringstream softBodyObjects, jointObjects, contactPointCounts, collisionPairCounts;
        std::stringstream memoryUsage, gpuMemoryUsage;
        std::stringstream gpuMemParticles, gpuMemSoftBodies, gpuMemFEMCloths, gpuMemHairSystems, gpuMemHeap;
        
        for (size_t i = 0; i < frameHistory.size(); i++)
        {
            const auto& frame = frameHistory[i];
            
            if (i > 0) {
                frameIndices << ", ";
                frameDurations << ", ";
                physicsStepTimes << ", ";
                collisionTimes << ", ";
                solverTimes << ", ";
                integrateTimes << ", ";
                objectCounts << ", ";
                dynamicObjects << ", ";
                staticObjects << ", ";
                softBodyObjects << ", ";
                jointObjects << ", ";
                contactPointCounts << ", ";
                collisionPairCounts << ", ";
                memoryUsage << ", ";
                gpuMemoryUsage << ", ";
                gpuMemParticles << ", ";
                gpuMemSoftBodies << ", ";
                gpuMemFEMCloths << ", ";
                gpuMemHairSystems << ", ";
                gpuMemHeap << ", ";
            }
            
            frameIndices << i;
            frameDurations << frame.frameDuration;
            physicsStepTimes << frame.physicsStepTime;
            collisionTimes << frame.collisionDetectionTime;
            solverTimes << frame.solverTime;
            integrateTimes << frame.integrateTime;
            
            objectCounts << frame.activeObjects;
            dynamicObjects << frame.activeDynamicObjects;
            staticObjects << frame.activeStaticObjects;
            softBodyObjects << frame.activeSoftBodies;
            jointObjects << frame.activeJoints;
            
            contactPointCounts << frame.contactPoints;
            collisionPairCounts << frame.collisionPairs;
            
            memoryUsage << (frame.memoryUsage / (1024.0 * 1024.0)); // Convert to MB
            gpuMemoryUsage << (frame.totalGPUMemory / (1024.0 * 1024.0)); // Convert to MB
            
            gpuMemParticles << (frame.gpuMemParticles / (1024.0 * 1024.0));
            gpuMemSoftBodies << (frame.gpuMemSoftBodies / (1024.0 * 1024.0));
            gpuMemFEMCloths << (frame.gpuMemFEMCloths / (1024.0 * 1024.0));
            gpuMemHairSystems << (frame.gpuMemHairSystems / (1024.0 * 1024.0));
            gpuMemHeap << (frame.gpuMemHeap / (1024.0 * 1024.0));
        }
        
        // Generate HTML
        file << R"(<!DOCTYPE html>
<html>
<head>
    <title>Physics Engine Statistics</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .chart-container { width: 80%; margin: 20px auto; }
        .chart-wrapper { margin-bottom: 30px; }
        h1, h2 { color: #333; }
        .summary { background-color: #f5f5f5; padding: 15px; border-radius: 5px; margin-bottom: 20px; }
        .summary table { width: 100%; border-collapse: collapse; }
        .summary th, .summary td { padding: 8px; text-align: left; border-bottom: 1px solid #ddd; }
        .summary th { background-color: #eee; }
    </style>
</head>
<body>
    <h1>Physics Engine Statistics Report</h1>
    
    <div class="summary">
        <h2>Summary Statistics</h2>
        <table>
            <tr><th>Metric</th><th>Value</th></tr>
            <tr><td>Average Frame Time</td><td>)";
        
        file << statistic.GetAverageFrameTime() * 0.001f;
        
        file << R"( ms</td></tr>
            <tr><td>Average Physics Step Time</td><td>)";
            
        file << statistic.GetAveragePhysicsStepTime() * 0.001f;
        
        file << R"( ms</td></tr>
            <tr><td>Peak Frame Time</td><td>)";
            
        file << statistic.GetPeakFrameTime() * 0.001f;
        
        file << R"( ms</td></tr>
            <tr><td>Physics Time Percentage</td><td>)";
            
        file << statistic.GetPhysicsTimePercentage();
        
        file << R"(%</td></tr>
            <tr><td>Total Frames</td><td>)";
            
        file << frameHistory.size();
        
        file << R"(</td></tr>
        </table>
    </div>
    
    <div class="chart-wrapper">
        <h2>Frame Time and Physics Step Time</h2>
        <div class="chart-container">
            <canvas id="timeChart"></canvas>
        </div>
    </div>
    
    <div class="chart-wrapper">
        <h2>Physics Components Breakdown</h2>
        <div class="chart-container">
            <canvas id="componentsChart"></canvas>
        </div>
    </div>
    
    <div class="chart-wrapper">
        <h2>Object Counts</h2>
        <div class="chart-container">
            <canvas id="objectsChart"></canvas>
        </div>
    </div>
    
    <div class="chart-wrapper">
        <h2>Collision Statistics</h2>
        <div class="chart-container">
            <canvas id="collisionChart"></canvas>
        </div>
    </div>
    
    <div class="chart-wrapper">
        <h2>Memory Usage</h2>
        <div class="chart-container">
            <canvas id="memoryChart"></canvas>
        </div>
    </div>
    
    <div class="chart-wrapper">
        <h2>GPU Memory Breakdown</h2>
        <div class="chart-container">
            <canvas id="gpuMemoryChart"></canvas>
        </div>
    </div>
    
    <script>
        // Common chart configurations
        const commonOptions = {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            elements: { point: { radius: 0 } },
            scales: {
                x: { title: { display: true, text: 'Frame' } },
                y: { beginAtZero: true }
            }
        };
        
        // Create Charts
        const timeCtx = document.getElementById('timeChart').getContext('2d');
        const timeChart = new Chart(timeCtx, {
            type: 'line',
            data: {
                labels: [)";
        file << frameIndices.str();
        file << R"(],
                datasets: [{
                    label: 'Frame Time (ms)',
                    data: [)";
        file << frameDurations.str();
        file << R"(],
                    borderColor: 'rgb(75, 192, 192)',
                    tension: 0.1
                },
                {
                    label: 'Physics Step Time (ms)',
                    data: [)";
        file << physicsStepTimes.str();
        file << R"(],
                    borderColor: 'rgb(255, 99, 132)',
                    tension: 0.1
                }]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        beginAtZero: true,
                        title: { display: true, text: 'Time (ms)' }
                    }
                }
            }
        });
        
        const componentsCtx = document.getElementById('componentsChart').getContext('2d');
        const componentsChart = new Chart(componentsCtx, {
            type: 'line',
            data: {
                labels: [)";
        file << frameIndices.str();
        file << R"(],
                datasets: [{
                    label: 'Collision Detection (ms)',
                    data: [)";
        file << collisionTimes.str();
        file << R"(],
                    borderColor: 'rgb(255, 159, 64)',
                    tension: 0.1
                },
                {
                    label: 'Solver (ms)',
                    data: [)";
        file << solverTimes.str();
        file << R"(],
                    borderColor: 'rgb(54, 162, 235)',
                    tension: 0.1
                },
                {
                    label: 'Integration (ms)',
                    data: [)";
        file << integrateTimes.str();
        file << R"(],
                    borderColor: 'rgb(153, 102, 255)',
                    tension: 0.1
                }]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        beginAtZero: true,
                        title: { display: true, text: 'Time (ms)' }
                    }
                }
            }
        });
        
        const objectsCtx = document.getElementById('objectsChart').getContext('2d');
        const objectsChart = new Chart(objectsCtx, {
            type: 'line',
            data: {
                labels: [)";
        file << frameIndices.str();
        file << R"(],
                datasets: [{
                    label: 'Total Objects',
                    data: [)";
        file << objectCounts.str();
        file << R"(],
                    borderColor: 'rgb(75, 192, 192)',
                    tension: 0.1
                },
                {
                    label: 'Dynamic Objects',
                    data: [)";
        file << dynamicObjects.str();
        file << R"(],
                    borderColor: 'rgb(255, 99, 132)',
                    tension: 0.1
                },
                {
                    label: 'Static Objects',
                    data: [)";
        file << staticObjects.str();
        file << R"(],
                    borderColor: 'rgb(255, 159, 64)',
                    tension: 0.1
                },
                {
                    label: 'Soft Bodies',
                    data: [)";
        file << softBodyObjects.str();
        file << R"(],
                    borderColor: 'rgb(54, 162, 235)',
                    tension: 0.1
                },
                {
                    label: 'Joints',
                    data: [)";
        file << jointObjects.str();
        file << R"(],
                    borderColor: 'rgb(153, 102, 255)',
                    tension: 0.1
                }]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        beginAtZero: true,
                        title: { display: true, text: 'Count' }
                    }
                }
            }
        });
        
        const collisionCtx = document.getElementById('collisionChart').getContext('2d');
        const collisionChart = new Chart(collisionCtx, {
            type: 'line',
            data: {
                labels: [)";
        file << frameIndices.str();
        file << R"(],
                datasets: [{
                    label: 'Contact Points',
                    data: [)";
        file << contactPointCounts.str();
        file << R"(],
                    borderColor: 'rgb(255, 99, 132)',
                    tension: 0.1
                },
                {
                    label: 'Collision Pairs',
                    data: [)";
        file << collisionPairCounts.str();
        file << R"(],
                    borderColor: 'rgb(54, 162, 235)',
                    tension: 0.1
                }]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        beginAtZero: true,
                        title: { display: true, text: 'Count' }
                    }
                }
            }
        });
        
        const memoryCtx = document.getElementById('memoryChart').getContext('2d');
        const memoryChart = new Chart(memoryCtx, {
            type: 'line',
            data: {
                labels: [)";
        file << frameIndices.str();
        file << R"(],
                datasets: [{
                    label: 'CPU Memory Usage (MB)',
                    data: [)";
        file << memoryUsage.str();
        file << R"(],
                    borderColor: 'rgb(255, 99, 132)',
                    tension: 0.1
                },
                {
                    label: 'GPU Memory Usage (MB)',
                    data: [)";
        file << gpuMemoryUsage.str();
        file << R"(],
                    borderColor: 'rgb(54, 162, 235)',
                    tension: 0.1
                }]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        beginAtZero: true,
                        title: { display: true, text: 'Memory (MB)' }
                    }
                }
            }
        });
        
        const gpuMemoryCtx = document.getElementById('gpuMemoryChart').getContext('2d');
        const gpuMemoryChart = new Chart(gpuMemoryCtx, {
            type: 'line',
            data: {
                labels: [)";
        file << frameIndices.str();
        file << R"(],
                datasets: [{
                    label: 'Particles (MB)',
                    data: [)";
        file << gpuMemParticles.str();
        file << R"(],
                    borderColor: 'rgb(255, 99, 132)',
                    tension: 0.1
                },
                {
                    label: 'Soft Bodies (MB)',
                    data: [)";
        file << gpuMemSoftBodies.str();
        file << R"(],
                    borderColor: 'rgb(54, 162, 235)',
                    tension: 0.1
                },
                {
                    label: 'FEM Cloths (MB)',
                    data: [)";
        file << gpuMemFEMCloths.str();
        file << R"(],
                    borderColor: 'rgb(255, 159, 64)',
                    tension: 0.1
                },
                {
                    label: 'Hair Systems (MB)',
                    data: [)";
        file << gpuMemHairSystems.str();
        file << R"(],
                    borderColor: 'rgb(75, 192, 192)',
                    tension: 0.1
                },
                {
                    label: 'Heap Memory (MB)',
                    data: [)";
        file << gpuMemHeap.str();
        file << R"(],
                    borderColor: 'rgb(153, 102, 255)',
                    tension: 0.1
                }]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        beginAtZero: true,
                        title: { display: true, text: 'Memory (MB)' }
                    }
                }
            }
        });
        
    </script>
</body>
</html>)";

        return true;
    }

    std::string getTimestampString()
    {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    PhysicsProfiler* m_Profiler;
};

inline bool ExportPhysicsProfilerData(PhysicsProfiler* profiler, 
                                      const std::string& prefix,
                                      ProfileChartExportFormat format)
{
    if (!profiler)
        return false;
        
    PhysicsProfilerChart chart(profiler);
    
    std::stringstream ss;
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    ss << prefix << "_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    
    switch (format)
    {
        case ProfileChartExportFormat::CSV:
            ss << ".csv";
            break;
        case ProfileChartExportFormat::JSON:
            ss << ".json";
            break;
        case ProfileChartExportFormat::HTML:
            ss << ".html";
            break;
    }
    
    return chart.exportData(ss.str(), format);
}

// Export physics engine statistics data
inline bool ExportPhysicsStatisticsData(PhysicsProfiler* profiler,
                                        const std::string& prefix,
                                        ProfileChartExportFormat format)
{
    if (!profiler)
        return false;
        
    PhysicsProfilerChart chart(profiler);
    
    std::stringstream ss;
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    ss << prefix << "_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    
    switch (format)
    {
        case ProfileChartExportFormat::CSV:
            ss << ".csv";
            break;
        case ProfileChartExportFormat::JSON:
            ss << ".json";
            break;
        case ProfileChartExportFormat::HTML:
            ss << ".html";
            break;
    }
    
    return chart.exportStatisticsData(ss.str(), format);
} 