#pragma once
#include "PhysicsProfiler.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>

class PhysicsProfilerChart
{
public:
    enum class ExportFormat 
    {
        CSV,
        JSON,
        HTML
    };

    PhysicsProfilerChart(PhysicsProfiler* profiler) : m_Profiler(profiler) {}

    bool exportData(const std::string& filepath, ExportFormat format = ExportFormat::CSV)
    {
        if (!m_Profiler)
            return false;

        std::ofstream file(filepath);
        if (!file.is_open())
            return false;

        bool result = false;

        switch (format) 
        {
            case ExportFormat::CSV:
                result = exportToCSV(file);
                break;
            case ExportFormat::JSON:
                result = exportToJSON(file);
                break;
            case ExportFormat::HTML:
                result = exportToHTML(file);
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
            const EventStats& stat = pair.second;
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
            const EventStats& stat = pair.second;
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

    bool exportToHTML(std::ofstream& file)
    {
        const auto& stats = m_Profiler->getEventStats();
        
        std::stringstream labels, avgData, minData, maxData;
        for (const auto& pair : stats)
        {
            const std::string& name = pair.first;
            const EventStats& stat = pair.second;
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
            const EventStats& stat = pair.second;
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
                                      const std::string& prefix = "physics_profile",
                                      PhysicsProfilerChart::ExportFormat format = PhysicsProfilerChart::ExportFormat::HTML)
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
        case PhysicsProfilerChart::ExportFormat::CSV:
            ss << ".csv";
            break;
        case PhysicsProfilerChart::ExportFormat::JSON:
            ss << ".json";
            break;
        case PhysicsProfilerChart::ExportFormat::HTML:
            ss << ".html";
            break;
    }
    
    return chart.exportData(ss.str(), format);
} 