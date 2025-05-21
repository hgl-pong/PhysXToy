#pragma once
#include "PxPhysicsAPI.h"
#include "foundation/PxProfiler.h"
#include "pvd/PxPvd.h"
#include "pvd/PxPvdTransport.h"
#include "Physics/PhysicsTypes.h"
#include "Physics/PhysicsCommon.h"
#include "common/PxProfileZone.h"
#include <chrono>
#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>
#include <numeric>
#include <fstream>

#define PHYSX_PVD_HOST "127.0.0.1"

class PhysicsProfiler;
extern bool ExportPhysicsProfilerData(PhysicsProfiler* profiler,
    const std::string& prefix = "physics_profile",
    ProfileChartExportFormat format = ProfileChartExportFormat::HTML);

// Export physics engine statistics data
extern bool ExportPhysicsStatisticsData(PhysicsProfiler* profiler,
    const std::string& prefix = "physics_statistics",
    ProfileChartExportFormat format = ProfileChartExportFormat::HTML);

class PhysicsProfilerChart;

// Physics engine statistics class for recording and analyzing performance and other data
class PhysicsStatistic
{
public:
    // Moving average window size
    static constexpr size_t DEFAULT_WINDOW_SIZE = 120;
    
    PhysicsStatistic(size_t windowSize = DEFAULT_WINDOW_SIZE)
        : m_WindowSize(windowSize)
    {
        m_FrameHistory.reserve(windowSize);
    }
    
    // Start a new frame
    void BeginFrame()
    {
        m_CurrentFrame.frameStartTime = GetTimeMicroseconds();
    }
    
    // End current frame and record statistics
    void EndFrame()
    {
        uint64_t currentTime = GetTimeMicroseconds();
        m_CurrentFrame.frameEndTime = currentTime;
        m_CurrentFrame.frameDuration = m_CurrentFrame.frameEndTime - m_CurrentFrame.frameStartTime;
        
        // Add current frame to history
        m_FrameHistory.push_back(m_CurrentFrame);
        
        // If history exceeds window size, remove oldest frame
        if (m_FrameHistory.size() > m_WindowSize)
        {
            m_FrameHistory.erase(m_FrameHistory.begin());
        }
        
        auto memoryUsagePre = m_CurrentFrame.memoryUsage;

        // Reset current frame data for next frame
        m_CurrentFrame = PhysicsStatisticsData::FrameStats();

        m_CurrentFrame.memoryUsage = memoryUsagePre;
    }
    
    // Record the start of a physics engine stage
    void BeginPhysicsStage(const std::string& stageName)
    {
        m_StageStartTimes[stageName] = GetTimeMicroseconds();
    }
    
    // Record the end of a physics engine stage and update corresponding statistics
    void EndPhysicsStage(const std::string& stageName)
    {
        auto it = m_StageStartTimes.find(stageName);
        if (it != m_StageStartTimes.end())
        {
            uint64_t startTime = it->second;
            uint64_t endTime = GetTimeMicroseconds();
            uint64_t duration = endTime - startTime;
            
            // Update timer based on stage name
            if (stageName == "PhysicsStep")
            {
                m_CurrentFrame.physicsStepTime = duration;
            }
            else if (stageName == "CollisionDetection")
            {
                m_CurrentFrame.collisionDetectionTime = duration;
            }
            else if (stageName == "Solver")
            {
                m_CurrentFrame.solverTime = duration;
            }
            else if (stageName == "Integrate")
            {
                m_CurrentFrame.integrateTime = duration;
            }
            
            m_StageStartTimes.erase(it);
        }
    }
    
    // Set object count statistics for current frame
    void SetObjectCounts(uint32_t total, uint32_t dynamic, uint32_t staticObj, uint32_t softBodies, uint32_t joints)
    {
        m_CurrentFrame.activeObjects = total;
        m_CurrentFrame.activeDynamicObjects = dynamic;
        m_CurrentFrame.activeStaticObjects = staticObj;
        m_CurrentFrame.activeSoftBodies = softBodies;
        m_CurrentFrame.activeJoints = joints;
    }
    
    // Set collision statistics for current frame
    void SetCollisionStats(uint32_t contactPoints, uint32_t collisionPairs)
    {
        m_CurrentFrame.contactPoints = contactPoints;
        m_CurrentFrame.collisionPairs = collisionPairs;
    }
    
    // Set memory usage for current frame
    void SetMemoryUsage(uint64_t bytes)
    {
        m_CurrentFrame.memoryUsage = bytes;
    }
    
    // Get the latest frame statistics
    const PhysicsStatisticsData::FrameStats& GetLatestFrameStats() const
    {
        return m_FrameHistory.empty() ? m_CurrentFrame : m_FrameHistory.back();
    }
    
    // Get all frame history statistics
    const std::vector<PhysicsStatisticsData::FrameStats>& GetFrameHistory() const
    {
        return m_FrameHistory;
    }
    
    // Get average frame time (microseconds)
    uint64_t GetAverageFrameTime() const
    {
        if (m_FrameHistory.empty()) return 0;
        
        uint64_t sum = 0;
        for (const auto& frame : m_FrameHistory)
        {
            sum += frame.frameDuration;
        }
        return sum / m_FrameHistory.size();
    }
    
    // Get average physics step time (microseconds)
    uint64_t GetAveragePhysicsStepTime() const
    {
        if (m_FrameHistory.empty()) return 0;
        
        uint64_t sum = 0;
        for (const auto& frame : m_FrameHistory)
        {
            sum += frame.physicsStepTime;
        }
        return sum / m_FrameHistory.size();
    }
    
    // Get peak frame time (microseconds)
    uint64_t GetPeakFrameTime() const
    {
        uint64_t peak = 0;
        for (const auto& frame : m_FrameHistory)
        {
            peak = std::max(peak, frame.frameDuration);
        }
        return peak;
    }
    
    // Get percentage of frame time spent in physics processing
    float GetPhysicsTimePercentage() const
    {
        if (m_FrameHistory.empty()) return 0.0f;
        
        uint64_t totalFrameTime = 0;
        uint64_t totalPhysicsTime = 0;
        
        for (const auto& frame : m_FrameHistory)
        {
            totalFrameTime += frame.frameDuration;
            totalPhysicsTime += frame.physicsStepTime;
        }
        
        if (totalFrameTime == 0) return 0.0f;
        return (float)totalPhysicsTime / totalFrameTime * 100.0f;
    }
    
    // Print detailed statistics
    void PrintDetailedStats() const
    {
        PHYSICS_PRINT("\n==== Physics Engine Detailed Statistics ====\n");
        
        // Frame time statistics
        const auto& latest = GetLatestFrameStats();
        PHYSICS_PRINT("Frame Time: %.2f ms\n", latest.frameDuration / 1000.0);
        PHYSICS_PRINT("Average Frame Time: %.2f ms\n", GetAverageFrameTime() / 1000.0);
        PHYSICS_PRINT("Peak Frame Time: %.2f ms\n", GetPeakFrameTime() / 1000.0);
        
        PHYSICS_PRINT("\nPhysics Time: %.2f ms (%.1f%% of frame)\n", 
            latest.physicsStepTime / 1000.0, 
            GetPhysicsTimePercentage());
        
        PHYSICS_PRINT("  Collision Detection: %.2f ms\n", latest.collisionDetectionTime / 1000.0);
        PHYSICS_PRINT("  Solver: %.2f ms\n", latest.solverTime / 1000.0);
        PHYSICS_PRINT("  Integration: %.2f ms\n", latest.integrateTime / 1000.0);
        
        PHYSICS_PRINT("\nObject Counts:\n");
        PHYSICS_PRINT("  Total Objects: %u\n", latest.activeObjects);
        PHYSICS_PRINT("  Dynamic Objects: %u\n", latest.activeDynamicObjects);
        PHYSICS_PRINT("  Static Objects: %u\n", latest.activeStaticObjects);
        PHYSICS_PRINT("  Soft Bodies: %u\n", latest.activeSoftBodies);
        PHYSICS_PRINT("  Joints: %u\n", latest.activeJoints);
        
        PHYSICS_PRINT("\nCollision Statistics:\n");
        PHYSICS_PRINT("  Contact Points: %u\n", latest.contactPoints);
        PHYSICS_PRINT("  Collision Pairs: %u\n", latest.collisionPairs);
        
        // Print memory usage
        PHYSICS_PRINT("\nMemory Usage:\n");
        PHYSICS_PRINT("  CPU Memory: %.2f MB\n", latest.memoryUsage / (1024.0 * 1024.0));
        PHYSICS_PRINT("  GPU Memory: %.2f MB\n", latest.totalGPUMemory / (1024.0 * 1024.0));
        
        // Print GPU memory details
        if (latest.totalGPUMemory > 0)
        {
            PHYSICS_PRINT("\nGPU Memory Details:\n");
            PHYSICS_PRINT("  Particles: %.2f MB\n", latest.gpuMemParticles / (1024.0 * 1024.0));
            PHYSICS_PRINT("  Soft Bodies: %.2f MB\n", latest.gpuMemSoftBodies / (1024.0 * 1024.0));
            PHYSICS_PRINT("  FEM Cloths: %.2f MB\n", latest.gpuMemFEMCloths / (1024.0 * 1024.0));
            PHYSICS_PRINT("  Hair Systems: %.2f MB\n", latest.gpuMemHairSystems / (1024.0 * 1024.0));
            PHYSICS_PRINT("  Heap Memory: %.2f MB\n", latest.gpuMemHeap / (1024.0 * 1024.0));
            
            PHYSICS_PRINT("\nGPU Heap Details:\n");
            PHYSICS_PRINT("  Broad Phase: %.2f MB\n", latest.gpuMemHeapBroadPhase / (1024.0 * 1024.0));
            PHYSICS_PRINT("  Narrow Phase: %.2f MB\n", latest.gpuMemHeapNarrowPhase / (1024.0 * 1024.0));
            PHYSICS_PRINT("  Solver: %.2f MB\n", latest.gpuMemHeapSolver / (1024.0 * 1024.0));
            PHYSICS_PRINT("  Articulation: %.2f MB\n", latest.gpuMemHeapArticulation / (1024.0 * 1024.0));
            PHYSICS_PRINT("  Simulation: %.2f MB\n", latest.gpuMemHeapSimulation / (1024.0 * 1024.0));
        }
        
        PHYSICS_PRINT("==========================================\n");
    }
    
    // Reset all statistics
    void Reset()
    {
        m_FrameHistory.clear();
        m_CurrentFrame = PhysicsStatisticsData::FrameStats();
        m_StageStartTimes.clear();
    }
    
    // Get current frame statistics (mutable)
    PhysicsStatisticsData::FrameStats& GetCurrentFrame()
    {
        return m_CurrentFrame;
    }
    
private:
    // Get current time (microseconds)
    uint64_t GetTimeMicroseconds() const 
    {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
    }
    
    size_t m_WindowSize;                                  // History window size
    PhysicsStatisticsData::FrameStats m_CurrentFrame;                            // Current frame statistics
    std::vector<PhysicsStatisticsData::FrameStats> m_FrameHistory;               // Frame history
    std::unordered_map<std::string, uint64_t> m_StageStartTimes; // Stage start times
};

class PhysicsProfiler : public physx::PxProfilerCallback, public IPhysicsProfiler
{
public:
	PhysicsProfiler(bool bEnablePVD = true, bool bEnableCustomProfiler = true)
		: m_bEnablePVD(bEnablePVD), 
		  m_bEnableCustomProfiler(bEnableCustomProfiler),
		  m_bCallPVDProfilingFunctions(bEnablePVD),
          m_Statistic(120) // Default 120 frames history
	{
		if (m_bEnablePVD)
		{
			m_Pvd = make_physx_ptr(PxCreatePvd(PxGetFoundation()));
			physx::PxPvdTransport *transport = physx::PxDefaultPvdSocketTransportCreate(PHYSX_PVD_HOST, 5425, 10);
			m_Pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
		}
		PxSetProfilerCallback(this);
	}

	virtual ~PhysicsProfiler() 
	{
		if (m_Pvd)
		{
			physx::PxPvdTransport *transport = m_Pvd->getTransport();
			m_Pvd->disconnect();
			m_Pvd.reset();
			PX_RELEASE(transport);
		}
	}

	virtual void* ZoneStart(const char* eventName, bool detached, uint64_t contextId) override
	{
		return zoneStart(eventName, detached, contextId);
	}
	virtual void ZoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) override
	{
		return zoneEnd(profilerData, eventName, detached, contextId);
	}
	virtual void RecordData(const char* name, float value, uint64_t contextId) override
	{
		if (m_bEnableCustomProfiler)
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			
			ProfileDataRecord record;
			record.name = name;
			record.floatValue = value;
			record.contextId = contextId;
			record.isFloat = true;
			
			m_DataRecords.push_back(record);
			
			if (m_bVerboseOutput)
				PHYSICS_PRINT("Data: %s = %f (context ID %llu)\n", name, value, contextId);
            
            // Update statistics based on data records
            if (strcmp(name, "ActiveObjects") == 0)
            {
                m_LastActiveObjects = static_cast<int32_t>(value);
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveDynamicObjects") == 0)
            {
                m_LastActiveDynamicObjects = static_cast<int32_t>(value);
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveStaticObjects") == 0)
            {
                m_LastActiveStaticObjects = static_cast<int32_t>(value);
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveSoftBodies") == 0)
            {
                m_LastActiveSoftBodies = static_cast<int32_t>(value);
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveJoints") == 0)
            {
                m_LastActiveJoints = static_cast<int32_t>(value);
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ContactPoints") == 0)
            {
                m_LastContactPoints = static_cast<int32_t>(value);
                UpdateCollisionStats();
            }
            else if (strcmp(name, "CollisionPairs") == 0)
            {
                m_LastCollisionPairs = static_cast<int32_t>(value);
                UpdateCollisionStats();
            }
            else if (strcmp(name, "MemoryUsage") == 0)
            {
                m_Statistic.SetMemoryUsage(static_cast<uint64_t>(value));
            }
            // PhysX Statistics
            else if (strcmp(name, "DiscreteContactPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().discreteContactPairs = value;
            }
            else if (strcmp(name, "CacheHitPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().cacheHitPairs = value;
            }
            else if (strcmp(name, "ContactPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().contactPairsWithContacts = value;
            }
            else if (strcmp(name, "NewPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().newPairs = value;
            }
            else if (strcmp(name, "LostPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().lostPairs = value;
            }
            else if (strcmp(name, "NewTouches") == 0)
            {
                m_Statistic.GetCurrentFrame().newTouches = value;
            }
            else if (strcmp(name, "LostTouches") == 0)
            {
                m_Statistic.GetCurrentFrame().lostTouches = value;
            }
            else if (strcmp(name, "Partitions") == 0)
            {
                m_Statistic.GetCurrentFrame().partitions = value;
            }
            else if (strcmp(name, "ActiveConstraints") == 0)
            {
                m_Statistic.GetCurrentFrame().activeConstraints = value;
            }
            else if (strcmp(name, "ActiveDynamicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().activeDynamicBodies = value;
            }
            else if (strcmp(name, "ActiveKinematicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().activeKinematicBodies = value;
            }
            else if (strcmp(name, "StaticBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().staticBodies = value;
            }
            else if (strcmp(name, "DynamicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().dynamicBodies = value;
            }
            else if (strcmp(name, "KinematicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().kinematicBodies = value;
            }
            else if (strcmp(name, "Aggregates") == 0)
            {
                m_Statistic.GetCurrentFrame().aggregates = value;
            }
            else if (strcmp(name, "Articulations") == 0)
            {
                m_Statistic.GetCurrentFrame().articulations = value;
            }
            else if (strcmp(name, "AxisSolverConstraints") == 0)
            {
                m_Statistic.GetCurrentFrame().axisSolverConstraints = value;
            }
            else if (strcmp(name, "CompressedContactSize") == 0)
            {
                m_Statistic.GetCurrentFrame().compressedContactSize = value;
            }
            else if (strcmp(name, "RequiredContactConstraintMemory") == 0)
            {
                m_Statistic.GetCurrentFrame().requiredContactConstraintMemory = value;
            }
            else if (strcmp(name, "PeakConstraintMemory") == 0)
            {
                m_Statistic.GetCurrentFrame().peakConstraintMemory = value;
            }
            else if (strcmp(name, "BroadPhaseAdds") == 0)
            {
                m_Statistic.GetCurrentFrame().broadphaseAdds = value;
            }
            else if (strcmp(name, "BroadPhaseRemoves") == 0)
            {
                m_Statistic.GetCurrentFrame().broadphaseRemoves = value;
            }
            else if (strcmp(name, "GPUMemParticles") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemParticles = value;
            }
            else if (strcmp(name, "GPUMemSoftBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemSoftBodies = value;
            }
            else if (strcmp(name, "GPUMemFEMCloths") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemFEMCloths = value;
            }
            else if (strcmp(name, "GPUMemHairSystems") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHairSystems = value;
            }
            else if (strcmp(name, "GPUMemHeap") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeap = value;
            }
            else if (strcmp(name, "GPUMemHeapBroadPhase") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapBroadPhase = value;
            }
            else if (strcmp(name, "GPUMemHeapNarrowPhase") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapNarrowPhase = value;
            }
            else if (strcmp(name, "GPUMemHeapSolver") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapSolver = value;
            }
            else if (strcmp(name, "GPUMemHeapArticulation") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapArticulation = value;
            }
            else if (strcmp(name, "GPUMemHeapSimulation") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapSimulation = value;
            }
            else if (strcmp(name, "TotalGPUMemory") == 0)
            {
                m_Statistic.GetCurrentFrame().totalGPUMemory = value;
            }
            else if (strcmp(name, "TotalGPUMemoryLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().totalGPUMemory;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "TotalGPUMemoryHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().totalGPUMemory;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemParticlesLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemParticles;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemParticlesHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemParticles;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemSoftBodiesLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemSoftBodies;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemSoftBodiesHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemSoftBodies;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemFEMClothsLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemFEMCloths;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemFEMClothsHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemFEMCloths;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemHairSystemsLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHairSystems;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemHairSystemsHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHairSystems;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemHeapLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHeap;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemHeapHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHeap;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
		}

        //return recordData(name, value, contextId);
	}

	virtual void RecordData(const char* name, int32_t value, uint64_t contextId) override
	{
		if (m_bEnableCustomProfiler)
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			
			ProfileDataRecord record;
			record.name = name;
			record.intValue = value;
			record.contextId = contextId;
			record.isFloat = false;
			
			m_DataRecords.push_back(record);
			
			if (m_bVerboseOutput)
				PHYSICS_PRINT("Data: %s = %d (context ID %llu)\n", name, value, contextId);
            
            // Update statistics based on data records
            if (strcmp(name, "ActiveObjects") == 0)
            {
                m_LastActiveObjects = value;
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveDynamicObjects") == 0)
            {
                m_LastActiveDynamicObjects = value;
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveStaticObjects") == 0)
            {
                m_LastActiveStaticObjects = value;
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveSoftBodies") == 0)
            {
                m_LastActiveSoftBodies = value;
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ActiveJoints") == 0)
            {
                m_LastActiveJoints = value;
                UpdateObjectCounts();
            }
            else if (strcmp(name, "ContactPoints") == 0)
            {
                m_LastContactPoints = value;
                UpdateCollisionStats();
            }
            else if (strcmp(name, "CollisionPairs") == 0)
            {
                m_LastCollisionPairs = value;
                UpdateCollisionStats();
            }
            else if (strcmp(name, "MemoryUsage") == 0)
            {
                m_Statistic.SetMemoryUsage(static_cast<uint64_t>(value));
            }
            // PhysX统计数据处理
            else if (strcmp(name, "DiscreteContactPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().discreteContactPairs = value;
            }
            else if (strcmp(name, "CacheHitPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().cacheHitPairs = value;
            }
            else if (strcmp(name, "ContactPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().contactPairsWithContacts = value;
            }
            else if (strcmp(name, "NewPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().newPairs = value;
            }
            else if (strcmp(name, "LostPairs") == 0)
            {
                m_Statistic.GetCurrentFrame().lostPairs = value;
            }
            else if (strcmp(name, "NewTouches") == 0)
            {
                m_Statistic.GetCurrentFrame().newTouches = value;
            }
            else if (strcmp(name, "LostTouches") == 0)
            {
                m_Statistic.GetCurrentFrame().lostTouches = value;
            }
            else if (strcmp(name, "Partitions") == 0)
            {
                m_Statistic.GetCurrentFrame().partitions = value;
            }
            else if (strcmp(name, "ActiveConstraints") == 0)
            {
                m_Statistic.GetCurrentFrame().activeConstraints = value;
            }
            else if (strcmp(name, "ActiveDynamicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().activeDynamicBodies = value;
            }
            else if (strcmp(name, "ActiveKinematicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().activeKinematicBodies = value;
            }
            else if (strcmp(name, "StaticBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().staticBodies = value;
            }
            else if (strcmp(name, "DynamicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().dynamicBodies = value;
            }
            else if (strcmp(name, "KinematicBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().kinematicBodies = value;
            }
            else if (strcmp(name, "Aggregates") == 0)
            {
                m_Statistic.GetCurrentFrame().aggregates = value;
            }
            else if (strcmp(name, "Articulations") == 0)
            {
                m_Statistic.GetCurrentFrame().articulations = value;
            }
            else if (strcmp(name, "AxisSolverConstraints") == 0)
            {
                m_Statistic.GetCurrentFrame().axisSolverConstraints = value;
            }
            else if (strcmp(name, "CompressedContactSize") == 0)
            {
                m_Statistic.GetCurrentFrame().compressedContactSize = value;
            }
            else if (strcmp(name, "RequiredContactConstraintMemory") == 0)
            {
                m_Statistic.GetCurrentFrame().requiredContactConstraintMemory = value;
            }
            else if (strcmp(name, "PeakConstraintMemory") == 0)
            {
                m_Statistic.GetCurrentFrame().peakConstraintMemory = value;
            }
            else if (strcmp(name, "BroadPhaseAdds") == 0)
            {
                m_Statistic.GetCurrentFrame().broadphaseAdds = value;
            }
            else if (strcmp(name, "BroadPhaseRemoves") == 0)
            {
                m_Statistic.GetCurrentFrame().broadphaseRemoves = value;
            }
            else if (strcmp(name, "GPUMemParticles") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemParticles = value;
            }
            else if (strcmp(name, "GPUMemSoftBodies") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemSoftBodies = value;
            }
            else if (strcmp(name, "GPUMemFEMCloths") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemFEMCloths = value;
            }
            else if (strcmp(name, "GPUMemHairSystems") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHairSystems = value;
            }
            else if (strcmp(name, "GPUMemHeap") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeap = value;
            }
            else if (strcmp(name, "GPUMemHeapBroadPhase") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapBroadPhase = value;
            }
            else if (strcmp(name, "GPUMemHeapNarrowPhase") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapNarrowPhase = value;
            }
            else if (strcmp(name, "GPUMemHeapSolver") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapSolver = value;
            }
            else if (strcmp(name, "GPUMemHeapArticulation") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapArticulation = value;
            }
            else if (strcmp(name, "GPUMemHeapSimulation") == 0)
            {
                m_Statistic.GetCurrentFrame().gpuMemHeapSimulation = value;
            }
            else if (strcmp(name, "TotalGPUMemory") == 0)
            {
                m_Statistic.GetCurrentFrame().totalGPUMemory = value;
            }
            else if (strcmp(name, "TotalGPUMemoryLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().totalGPUMemory;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "TotalGPUMemoryHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().totalGPUMemory;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemParticlesLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemParticles;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemParticlesHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemParticles;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemSoftBodiesLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemSoftBodies;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemSoftBodiesHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemSoftBodies;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemFEMClothsLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemFEMCloths;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemFEMClothsHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemFEMCloths;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemHairSystemsLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHairSystems;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemHairSystemsHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHairSystems;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
            else if (strcmp(name, "GPUMemHeapLow") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHeap;
                totalValue = (totalValue & 0xFFFFFFFF00000000) | (uint32_t)value;
            }
            else if (strcmp(name, "GPUMemHeapHigh") == 0)
            {
                uint64_t& totalValue = m_Statistic.GetCurrentFrame().gpuMemHeap;
                totalValue = (totalValue & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
            }
		}
	}

	virtual void RecordFrame(const char* name, uint64_t contextId) override
	{
		if (m_bEnableCustomProfiler)
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			
			if (m_bVerboseOutput)
				PHYSICS_PRINT("Frame: %s (context ID %llu)\n", name, contextId);
            
            // Begin a new frame in the statistics object
            m_Statistic.BeginFrame();
		}
	}

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId) override
	{
		if (m_bEnableCustomProfiler)
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			uint64_t currentTime = getCurrentTimeMicroseconds();
			
			ProfileTimingEvent event;
			event.name = eventName;
			event.startTime = currentTime;
			event.contextId = contextId;
		
			m_ActiveEvents[contextId] = event;
			
			if (m_bVerboseOutput)
				PHYSICS_PRINT("Start: %s (context ID %llu) @ %llu us\n", eventName, contextId, currentTime);
            
            // Record physics engine stage start
            if (strcmp(eventName, "PhysicsStep") == 0) {
                m_Statistic.BeginPhysicsStage("PhysicsStep");
            }
            else if (strcmp(eventName, "CollisionDetection") == 0) {
                m_Statistic.BeginPhysicsStage("CollisionDetection");
            }
            else if (strcmp(eventName, "Solver") == 0) {
                m_Statistic.BeginPhysicsStage("Solver");
            }
            else if (strcmp(eventName, "Integrate") == 0) {
                m_Statistic.BeginPhysicsStage("Integrate");
            }
		}

		return m_bCallPVDProfilingFunctions ? m_Pvd->zoneStart(eventName, detached, contextId) : nullptr;
	}

	virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) override
	{
		if (m_bCallPVDProfilingFunctions)
			m_Pvd->zoneEnd(profilerData, eventName, detached, contextId);
			
		if (m_bEnableCustomProfiler)
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			uint64_t currentTime = getCurrentTimeMicroseconds();
			
			auto it = m_ActiveEvents.find(contextId);
			if (it != m_ActiveEvents.end())
			{
				ProfileTimingEvent& event = it->second;
				event.endTime = currentTime;
				event.duration = currentTime - event.startTime;
				
				m_CompletedEvents.push_back(event);
				
				auto statIt = m_EventStats.find(event.name);
				if (statIt != m_EventStats.end()) {
					statIt->second.totalTime += event.duration;
					statIt->second.count++;
					statIt->second.maxTime = std::max(statIt->second.maxTime, event.duration);
					statIt->second.minTime = std::min(statIt->second.minTime, event.duration);
				} else {
                    ProfileEventStats stats;
					stats.totalTime = event.duration;
					stats.count = 1;
					stats.maxTime = event.duration;
					stats.minTime = event.duration;
					m_EventStats[event.name] = stats;
				}
				
				m_ActiveEvents.erase(it);
				
				if (m_bVerboseOutput)
					PHYSICS_PRINT("End: %s (context ID %llu) @ %llu us, duration: %llu us\n", 
						eventName, contextId, currentTime, event.duration);
                
                // Record physics engine stage end
                if (strcmp(eventName, "PhysicsStep") == 0) {
                    m_Statistic.EndPhysicsStage("PhysicsStep");
                }
                else if (strcmp(eventName, "CollisionDetection") == 0) {
                    m_Statistic.EndPhysicsStage("CollisionDetection");
                }
                else if (strcmp(eventName, "Solver") == 0) {
                    m_Statistic.EndPhysicsStage("Solver");
                }
                else if (strcmp(eventName, "Integrate") == 0) {
                    m_Statistic.EndPhysicsStage("Integrate");
                }
			}
		}
	}

	void PrintStatistics() const
	{
		if (!m_bEnableCustomProfiler) 
			return;
			
		PHYSICS_PRINT("==================================\n");
		PHYSICS_PRINT("Physics Profiler Statistics\n");
		PHYSICS_PRINT("==================================\n");
		PHYSICS_PRINT("%-26s | %5s | %11s | %11s | %11s | %11s |\n",
			"Event Name", "Count", "Total Time", "Avg Time", "Min Time", "Max Time");
		PHYSICS_PRINT("---------------------------------+-------+-------------+-------------+-------------+-------------|\n");
		
		for (const auto& pair : m_EventStats)
		{
			const std::string& name = pair.first;
			const ProfileEventStats& stats = pair.second;
			double avgTime = stats.count > 0 ? static_cast<double>(stats.totalTime) / stats.count : 0.0;
			
			PHYSICS_PRINT("%-26s | %5u | %11llu | %11.2f | %11llu | %11llu |\n",
				name.c_str(), stats.count, stats.totalTime, avgTime, stats.minTime, stats.maxTime);
		}
		
		PHYSICS_PRINT("==================================\n");
	}
    
    // Print detailed physics engine statistics
    void PrintDetailedStatistics() const
    {
        if (!m_bEnableCustomProfiler) 
            return;
            
        // Call statistics object's print method
        m_Statistic.PrintDetailedStats();
	}

	void ResetStatistics()
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		m_EventStats.clear();
		m_CompletedEvents.clear();
		m_DataRecords.clear();
        
        // Reset detailed statistics
        m_Statistic.Reset();
        
        // Reset temporary counter cache
        m_LastActiveObjects = 0;
        m_LastActiveDynamicObjects = 0;
        m_LastActiveStaticObjects = 0;
        m_LastActiveSoftBodies = 0;
        m_LastActiveJoints = 0;
        m_LastContactPoints = 0;
        m_LastCollisionPairs = 0;
	}

	void SetVerboseOutput(bool enable) 
	{ 
		m_bVerboseOutput = enable; 
	}

	void EnableCustomProfiler(bool enable) 
	{ 
		m_bEnableCustomProfiler = enable; 
	}

	void EnablePVDProfiler(bool enable) 
	{ 
		m_bCallPVDProfilingFunctions = enable && m_bEnablePVD; 
	}

	physx::PxPvd* GetPVD()
	{
		return m_Pvd.get();
	}

	const std::unordered_map<std::string, ProfileEventStats>& getEventStats() const
	{
		return m_EventStats;
	}

	const std::vector<ProfileTimingEvent>& getCompletedEvents() const
	{
		return m_CompletedEvents;
	}

	const std::vector<ProfileDataRecord>& getDataRecords() const
	{
		return m_DataRecords;
	}
    
    // Get reference to statistics object
    PhysicsStatistic& GetStatistic()
    {
        return m_Statistic;
    }
    
    // Get const reference to statistics object
    const PhysicsStatistic& GetStatistic() const
    {
        return m_Statistic;
	}

	// IPhysicsProfiler interface implementation
	virtual const PhysicsStatisticsData::FrameStats& GetLatestFrameStats() const override
	{
		return m_Statistic.GetLatestFrameStats();
	}
	
	virtual const std::vector<PhysicsStatisticsData::FrameStats>& GetFrameHistory() const override
	{
		return m_Statistic.GetFrameHistory();
	}
	
	virtual uint64_t GetAverageFrameTime() const override
	{
		return m_Statistic.GetAverageFrameTime();
	}
	
	virtual uint64_t GetAveragePhysicsStepTime() const override
	{
		return m_Statistic.GetAveragePhysicsStepTime();
	}
	
	virtual uint64_t GetPeakFrameTime() const override
	{
		return m_Statistic.GetPeakFrameTime();
	}
	
	virtual float GetPhysicsTimePercentage() const override
	{
		return m_Statistic.GetPhysicsTimePercentage();
	}
	
	virtual const std::unordered_map<std::string, ProfileEventStats>& GetEventStats() const override
	{
		return reinterpret_cast<const std::unordered_map<std::string, ProfileEventStats>&>(m_EventStats);
	}
	
	virtual bool ExportStatisticsToCSV(const std::string& filename) override
	{
		return ExportPhysicsStatisticsData(this, filename, ProfileChartExportFormat::CSV);
	}
	
	virtual bool ExportStatisticsToJSON(const std::string& filename) override
	{
		return ExportPhysicsStatisticsData(this, filename, ProfileChartExportFormat::JSON);
	}
	
	virtual bool ExportStatisticsToHTML(const std::string& filename) override
	{
		return ExportPhysicsStatisticsData(this, filename, ProfileChartExportFormat::HTML);
	}

    virtual void EndFrame() override
    {
        m_Statistic.EndFrame();
        
        UpdateObjectCounts();
        UpdateCollisionStats();
        
    }

	virtual void SetMemoryUsage(uint64_t bytes) override
	{
		m_Statistic.SetMemoryUsage(bytes);
	}

	virtual void AddMemoryUsage(uint64_t bytes) override
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		m_Statistic.GetCurrentFrame().memoryUsage += bytes;
	}
	
	virtual void SubtractMemoryUsage(uint64_t bytes) override
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		auto& currentMemory = m_Statistic.GetCurrentFrame().memoryUsage;
		if (currentMemory >= bytes) {
			currentMemory -= bytes;
		} else {
			currentMemory = 0;
		}
	}
	
	virtual uint64_t GetMemoryUsage() const override
	{
		return m_Statistic.GetLatestFrameStats().memoryUsage;
	}

private:
	uint64_t getCurrentTimeMicroseconds() const 
	{
		auto now = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(
			now.time_since_epoch()).count();
	}
    
    // Update object count statistics
    void UpdateObjectCounts()
    {
        m_Statistic.SetObjectCounts(
            m_LastActiveObjects,
            m_LastActiveDynamicObjects,
            m_LastActiveStaticObjects,
            m_LastActiveSoftBodies,
            m_LastActiveJoints
        );
    }
    
    // Update collision statistics
    void UpdateCollisionStats()
    {
        m_Statistic.SetCollisionStats(
            m_LastContactPoints,
            m_LastCollisionPairs
        );
	}

	bool m_bEnablePVD = true;
	bool m_bEnableCustomProfiler = true;
	bool m_bVerboseOutput = false;
	bool m_bCallPVDProfilingFunctions = true;
	PhysXPtr<physx::PxPvd> m_Pvd;
	
	std::mutex m_Mutex;
	std::unordered_map<uint64_t, ProfileTimingEvent> m_ActiveEvents;
	std::vector<ProfileTimingEvent> m_CompletedEvents;
	std::vector<ProfileDataRecord> m_DataRecords;
	std::unordered_map<std::string, ProfileEventStats> m_EventStats;
    
    // Detailed statistics object
    PhysicsStatistic m_Statistic;
    
    // Temporary counter cache
    int32_t m_LastActiveObjects = 0;
    int32_t m_LastActiveDynamicObjects = 0;
    int32_t m_LastActiveStaticObjects = 0;
    int32_t m_LastActiveSoftBodies = 0;
    int32_t m_LastActiveJoints = 0;
    int32_t m_LastContactPoints = 0;
    int32_t m_LastCollisionPairs = 0;
};

#ifdef _DEBUG
    #define ENABLE_PHYSICS_PROFILER
#endif

#ifdef ENABLE_PHYSICS_PROFILER
	#define PHYSICS_PROFILE_ZONE(x, y)										\
		PhysicsProfilerScope PHYSICS_CONCAT(_scoped, __LINE__)(x, false, (size_t)y)
	#define PHYSICS_PROFILE_START_CROSSTHREAD(x, y)							\
		if(PhysicsEngineUtils::GetProfiler())										\
			PhysicsEngineUtils::GetProfiler()->ZoneStart(x, true, (size_t)y)
	#define PHYSICS_PROFILE_STOP_CROSSTHREAD(x, y)							\
		if(PhysicsEngineUtils::GetProfiler())										\
			PhysicsEngineUtils::GetProfiler()->ZoneEnd(NULL, x, true, (size_t)y)
	#define PHYSICS_PROFILE_VALUE(x, y, z)									\
		if(PhysicsEngineUtils::GetProfiler())										\
			PhysicsEngineUtils::GetProfiler()->RecordData(x, y, (size_t)z)
	#define PHYSICS_PROFILE_FRAME(x, y)                                                                                                         \
		if(PhysicsEngineUtils::GetProfiler())                                                                                                        \
			PhysicsEngineUtils::GetProfiler()->RecordFrame(x, (size_t)y)
	#define PHYSICS_PROFILE_END_FRAME()                                                                                                         \
		if(PhysicsEngineUtils::GetProfiler())                                                                                                        \
			PhysicsEngineUtils::GetProfiler()->EndFrame()
    #define PHYSICS_PROFILE_ADD_MEMORY_USAGE(x)                                                                                                         \
		if(PhysicsEngineUtils::GetProfiler())                                                                                                        \
			PhysicsEngineUtils::GetProfiler()->AddMemoryUsage(x)
    #define PHYSICS_PROFILE_SUBTRACT_MEMORY_USAGE(x)                                                                                                         \
		if(PhysicsEngineUtils::GetProfiler())                                                                                                        \
			PhysicsEngineUtils::GetProfiler()->SubtractMemoryUsage(x)
#else
	#define PHYSICS_PROFILE_ZONE(x, y)
	#define PHYSICS_PROFILE_START_CROSSTHREAD(x, y)
	#define PHYSICS_PROFILE_STOP_CROSSTHREAD(x, y)
	#define PHYSICS_PROFILE_VALUE(x, y, z)
	#define PHYSICS_PROFILE_FRAME(x, y)
	#define PHYSICS_PROFILE_END_FRAME()
	#define PHYSICS_PROFILE_ADD_MEMORY_USAGE(x)
	#define PHYSICS_PROFILE_SUBTRACT_MEMORY_USAGE(x)
#endif

#include "PhysicsProfilerChart.h"