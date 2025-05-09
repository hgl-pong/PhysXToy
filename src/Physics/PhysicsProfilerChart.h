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
             << "ContactPoints,CollisionPairs,MemoryUsage\n";
        
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
                 << frame.memoryUsage << "\n";
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
                 << "        \"memoryUsage\": " << frame.memoryUsage << "\n"
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
        
        // Prepare frame time data
        std::stringstream frameIndices, frameDurations, physicsStepTimes, collisionTimes, solverTimes, integrateTimes;
        
        // Prepare object count data
        std::stringstream objectCounts, dynamicObjects, staticObjects, softBodyObjects, jointObjects;
        
        // Prepare collision statistics data
        std::stringstream contactPointCounts, collisionPairCounts;
        
        // Prepare memory usage data
        std::stringstream memoryUsage;
        
        // Fill data
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
        }
        
        // Generate HTML
        file << "<!DOCTYPE html>\n"
             << "<html>\n"
             << "<head>\n"
             << "  <title>Physics Engine Detailed Statistics</title>\n"
             << "  <script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>\n"
             << "  <style>\n"
             << "    body { font-family: Arial, sans-serif; margin: 20px; }\n"
             << "    .chart-container { width: 800px; height: 400px; margin: 20px auto; }\n"
             << "    .stats-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin: 20px auto; max-width: 800px; }\n"
             << "    .stats-box { background-color: #f8f9fa; border-radius: 5px; padding: 15px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }\n"
             << "    .stats-heading { margin-top: 0; color: #333; }\n"
             << "    .stats-value { font-size: 24px; font-weight: bold; margin: 10px 0; color: #007bff; }\n"
             << "    .stats-label { color: #6c757d; font-size: 14px; }\n"
             << "    table { width: 800px; margin: 20px auto; border-collapse: collapse; }\n"
             << "    th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }\n"
             << "    th { background-color: #f2f2f2; }\n"
             << "    tr:nth-child(even) { background-color: #f9f9f9; }\n"
             << "    h2 { text-align: center; color: #333; margin-top: 40px; }\n"
             << "  </style>\n"
             << "</head>\n"
             << "<body>\n"
             << "  <h1 style=\"text-align: center;\">Physics Engine Detailed Statistics</h1>\n"
             
             // Summary statistics
             << "  <div class=\"stats-grid\">\n"
             << "    <div class=\"stats-box\">\n"
             << "      <h3 class=\"stats-heading\">Average Frame Time</h3>\n"
             << "      <div class=\"stats-value\">" << statistic.GetAverageFrameTime() << " us</div>\n"
             << "      <div class=\"stats-label\">Average time per frame</div>\n"
             << "    </div>\n"
             << "    <div class=\"stats-box\">\n"
             << "      <h3 class=\"stats-heading\">Peak Frame Time</h3>\n"
             << "      <div class=\"stats-value\">" << statistic.GetPeakFrameTime() << " us</div>\n"
             << "      <div class=\"stats-label\">Maximum frame time</div>\n"
             << "    </div>\n"
             << "    <div class=\"stats-box\">\n"
             << "      <h3 class=\"stats-heading\">Average Physics Step Time</h3>\n"
             << "      <div class=\"stats-value\">" << statistic.GetAveragePhysicsStepTime() << " us</div>\n"
             << "      <div class=\"stats-label\">Average time spent in physics simulation</div>\n"
             << "    </div>\n"
             << "    <div class=\"stats-box\">\n"
             << "      <h3 class=\"stats-heading\">Physics Time Percentage</h3>\n"
             << "      <div class=\"stats-value\">" << statistic.GetPhysicsTimePercentage() << "%</div>\n"
             << "      <div class=\"stats-label\">Percentage of frame time spent in physics</div>\n"
             << "    </div>\n"
             << "  </div>\n"
             
             // Frame time chart
             << "  <h2>Frame Time Analysis</h2>\n"
             << "  <div class=\"chart-container\">\n"
             << "    <canvas id=\"frameTimeChart\"></canvas>\n"
             << "  </div>\n"
             
             // Physics stage time chart
             << "  <h2>Physics Stage Time Analysis</h2>\n"
             << "  <div class=\"chart-container\">\n"
             << "    <canvas id=\"physicsStageChart\"></canvas>\n"
             << "  </div>\n"
             
             // Object count chart
             << "  <h2>Object Count Analysis</h2>\n"
             << "  <div class=\"chart-container\">\n"
             << "    <canvas id=\"objectCountChart\"></canvas>\n"
             << "  </div>\n"
             
             // Collision statistics chart
             << "  <h2>Collision Statistics</h2>\n"
             << "  <div class=\"chart-container\">\n"
             << "    <canvas id=\"collisionChart\"></canvas>\n"
             << "  </div>\n"
             
             // Memory usage chart
             << "  <h2>Memory Usage</h2>\n"
             << "  <div class=\"chart-container\">\n"
             << "    <canvas id=\"memoryChart\"></canvas>\n"
             << "  </div>\n"
             
             // Table data
             << "  <h2>Latest Frame Details</h2>\n"
             << "  <table>\n"
             << "    <tr><th>Metric</th><th>Value</th></tr>\n";
        
        // Only show latest frame detailed data
        if (!frameHistory.empty())
        {
            const auto& latest = statistic.GetLatestFrameStats();
            
            file << "    <tr><td>Frame Duration</td><td>" << latest.frameDuration << " us</td></tr>\n"
                 << "    <tr><td>Physics Step Time</td><td>" << latest.physicsStepTime << " us</td></tr>\n"
                 << "    <tr><td>Collision Detection Time</td><td>" << latest.collisionDetectionTime << " us</td></tr>\n"
                 << "    <tr><td>Solver Time</td><td>" << latest.solverTime << " us</td></tr>\n"
                 << "    <tr><td>Integration Time</td><td>" << latest.integrateTime << " us</td></tr>\n"
                 << "    <tr><td>Active Objects Total</td><td>" << latest.activeObjects << "</td></tr>\n"
                 << "    <tr><td>Active Dynamic Objects</td><td>" << latest.activeDynamicObjects << "</td></tr>\n"
                 << "    <tr><td>Active Static Objects</td><td>" << latest.activeStaticObjects << "</td></tr>\n"
                 << "    <tr><td>Active Soft Bodies</td><td>" << latest.activeSoftBodies << "</td></tr>\n"
                 << "    <tr><td>Active Joints</td><td>" << latest.activeJoints << "</td></tr>\n"
                 << "    <tr><td>Contact Points</td><td>" << latest.contactPoints << "</td></tr>\n"
                 << "    <tr><td>Collision Pairs</td><td>" << latest.collisionPairs << "</td></tr>\n"
                 << "    <tr><td>Memory Usage</td><td>" << (latest.memoryUsage / (1024.0 * 1024.0)) << " MB</td></tr>\n";
        }
        
        file << "  </table>\n"
             // JavaScript chart code
             << "  <script>\n"
             // Frame time chart
             << "    const frameTimeCtx = document.getElementById('frameTimeChart');\n"
             << "    new Chart(frameTimeCtx, {\n"
             << "      type: 'line',\n"
             << "      data: {\n"
             << "        labels: [" << frameIndices.str() << "],\n"
             << "        datasets: [{\n"
             << "          label: 'Frame Duration',\n"
             << "          data: [" << frameDurations.str() << "],\n"
             << "          borderColor: 'rgb(54, 162, 235)',\n"
             << "          backgroundColor: 'rgba(54, 162, 235, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Physics Step Time',\n"
             << "          data: [" << physicsStepTimes.str() << "],\n"
             << "          borderColor: 'rgb(255, 99, 132)',\n"
             << "          backgroundColor: 'rgba(255, 99, 132, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }]\n"
             << "      },\n"
             << "      options: {\n"
             << "        responsive: true,\n"
             << "        plugins: {\n"
             << "          title: { display: true, text: 'Frame Time vs Physics Time (microseconds)' }\n"
             << "        },\n"
             << "        scales: { y: { beginAtZero: true } }\n"
             << "      }\n"
             << "    });\n"
             
             // Physics stage time chart
             << "    const physicsStageCtx = document.getElementById('physicsStageChart');\n"
             << "    new Chart(physicsStageCtx, {\n"
             << "      type: 'line',\n"
             << "      data: {\n"
             << "        labels: [" << frameIndices.str() << "],\n"
             << "        datasets: [{\n"
             << "          label: 'Collision Detection',\n"
             << "          data: [" << collisionTimes.str() << "],\n"
             << "          borderColor: 'rgb(75, 192, 192)',\n"
             << "          backgroundColor: 'rgba(75, 192, 192, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Solver',\n"
             << "          data: [" << solverTimes.str() << "],\n"
             << "          borderColor: 'rgb(153, 102, 255)',\n"
             << "          backgroundColor: 'rgba(153, 102, 255, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Integration',\n"
             << "          data: [" << integrateTimes.str() << "],\n"
             << "          borderColor: 'rgb(255, 159, 64)',\n"
             << "          backgroundColor: 'rgba(255, 159, 64, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }]\n"
             << "      },\n"
             << "      options: {\n"
             << "        responsive: true,\n"
             << "        plugins: {\n"
             << "          title: { display: true, text: 'Physics Stage Times (microseconds)' }\n"
             << "        },\n"
             << "        scales: { y: { beginAtZero: true } }\n"
             << "      }\n"
             << "    });\n"
             
             // Object count chart
             << "    const objectCountCtx = document.getElementById('objectCountChart');\n"
             << "    new Chart(objectCountCtx, {\n"
             << "      type: 'line',\n"
             << "      data: {\n"
             << "        labels: [" << frameIndices.str() << "],\n"
             << "        datasets: [{\n"
             << "          label: 'Total Objects',\n"
             << "          data: [" << objectCounts.str() << "],\n"
             << "          borderColor: 'rgb(54, 162, 235)',\n"
             << "          backgroundColor: 'rgba(54, 162, 235, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Dynamic Objects',\n"
             << "          data: [" << dynamicObjects.str() << "],\n"
             << "          borderColor: 'rgb(255, 99, 132)',\n"
             << "          backgroundColor: 'rgba(255, 99, 132, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Static Objects',\n"
             << "          data: [" << staticObjects.str() << "],\n"
             << "          borderColor: 'rgb(75, 192, 192)',\n"
             << "          backgroundColor: 'rgba(75, 192, 192, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Soft Bodies',\n"
             << "          data: [" << softBodyObjects.str() << "],\n"
             << "          borderColor: 'rgb(153, 102, 255)',\n"
             << "          backgroundColor: 'rgba(153, 102, 255, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Joints',\n"
             << "          data: [" << jointObjects.str() << "],\n"
             << "          borderColor: 'rgb(255, 159, 64)',\n"
             << "          backgroundColor: 'rgba(255, 159, 64, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }]\n"
             << "      },\n"
             << "      options: {\n"
             << "        responsive: true,\n"
             << "        plugins: {\n"
             << "          title: { display: true, text: 'Object Counts' }\n"
             << "        },\n"
             << "        scales: { y: { beginAtZero: true } }\n"
             << "      }\n"
             << "    });\n"
             
             // Collision statistics chart
             << "    const collisionCtx = document.getElementById('collisionChart');\n"
             << "    new Chart(collisionCtx, {\n"
             << "      type: 'line',\n"
             << "      data: {\n"
             << "        labels: [" << frameIndices.str() << "],\n"
             << "        datasets: [{\n"
             << "          label: 'Contact Points',\n"
             << "          data: [" << contactPointCounts.str() << "],\n"
             << "          borderColor: 'rgb(54, 162, 235)',\n"
             << "          backgroundColor: 'rgba(54, 162, 235, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }, {\n"
             << "          label: 'Collision Pairs',\n"
             << "          data: [" << collisionPairCounts.str() << "],\n"
             << "          borderColor: 'rgb(255, 99, 132)',\n"
             << "          backgroundColor: 'rgba(255, 99, 132, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }]\n"
             << "      },\n"
             << "      options: {\n"
             << "        responsive: true,\n"
             << "        plugins: {\n"
             << "          title: { display: true, text: 'Collision Statistics' }\n"
             << "        },\n"
             << "        scales: { y: { beginAtZero: true } }\n"
             << "      }\n"
             << "    });\n"
             
             // Memory usage chart
             << "    const memoryCtx = document.getElementById('memoryChart');\n"
             << "    new Chart(memoryCtx, {\n"
             << "      type: 'line',\n"
             << "      data: {\n"
             << "        labels: [" << frameIndices.str() << "],\n"
             << "        datasets: [{\n"
             << "          label: 'Memory Usage (MB)',\n"
             << "          data: [" << memoryUsage.str() << "],\n"
             << "          borderColor: 'rgb(54, 162, 235)',\n"
             << "          backgroundColor: 'rgba(54, 162, 235, 0.1)',\n"
             << "          fill: true,\n"
             << "          tension: 0.1\n"
             << "        }]\n"
             << "      },\n"
             << "      options: {\n"
             << "        responsive: true,\n"
             << "        plugins: {\n"
             << "          title: { display: true, text: 'Memory Usage (MB)' }\n"
             << "        },\n"
             << "        scales: { y: { beginAtZero: true } }\n"
             << "      }\n"
             << "    });\n"
             << "  </script>\n"
             << "</body>\n"
             << "</html>\n";
        
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