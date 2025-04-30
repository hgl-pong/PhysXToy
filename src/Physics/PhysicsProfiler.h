#pragma once
#include "PxPhysicsAPI.h"
#include "foundation/PxProfiler.h"
#include "pvd/PxPvd.h"
#include "pvd/PxPvdTransport.h"
#include "Physics/PhysicsTypes.h"
#include "common/PxProfileZone.h"
#include <chrono>
#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>

#define PHYSX_PVD_HOST "127.0.0.1"

struct ProfileTimingEvent
{
	std::string name;
	uint64_t startTime;
	uint64_t endTime;
	uint64_t duration;
	uint64_t contextId;
};

struct ProfileDataRecord
{
	std::string name;
	union {
		int32_t intValue;
		float floatValue;
	};
	uint64_t contextId;
	bool isFloat;
};

struct EventStats {
	uint32_t count = 0;
	uint64_t totalTime = 0;
	uint64_t maxTime = 0;
	uint64_t minTime = UINT64_MAX;
};

class PhysicsProfiler : public physx::PxProfilerCallback
{
public:
	PhysicsProfiler(bool bEnablePVD = true, bool bEnableCustomProfiler = true)
		: m_bEnablePVD(bEnablePVD), 
		  m_bEnableCustomProfiler(bEnableCustomProfiler),
		  m_bCallPVDProfilingFunctions(bEnablePVD)
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
				printf("Start: %s (context ID %llu) @ %llu us\n", eventName, contextId, currentTime);
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
					EventStats stats;
					stats.totalTime = event.duration;
					stats.count = 1;
					stats.maxTime = event.duration;
					stats.minTime = event.duration;
					m_EventStats[event.name] = stats;
				}
				
				m_ActiveEvents.erase(it);
				
				if (m_bVerboseOutput)
					printf("End: %s (context ID %llu) @ %llu us, duration: %llu us\n", 
						eventName, contextId, currentTime, event.duration);
			}
		}
	}

	virtual void recordData(int32_t value, const char* valueName, uint64_t contextId)
	{
		if (m_bEnableCustomProfiler)
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			
			ProfileDataRecord record;
			record.name = valueName;
			record.intValue = value;
			record.contextId = contextId;
			record.isFloat = false;
			
			m_DataRecords.push_back(record);
			
			if (m_bVerboseOutput)
				printf("Data: %s (context ID %llu) = %d\n", valueName, contextId, value);
		}
	}

	virtual void recordData(float value, const char* valueName, uint64_t contextId)
	{
		if (m_bEnableCustomProfiler)
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			
			ProfileDataRecord record;
			record.name = valueName;
			record.floatValue = value;
			record.contextId = contextId;
			record.isFloat = true;
			
			m_DataRecords.push_back(record);
			
			if (m_bVerboseOutput)
				printf("Data: %s (context ID %llu) = %f\n", valueName, contextId, (double)value);
		}
	}

	virtual void recordFrame(const char* name, uint64_t contextId)
	{
		if (m_bEnableCustomProfiler && m_bVerboseOutput)
		{
			printf("Frame: %s (context ID %llu)\n", name, contextId);
		}
	}

	void printStatistics() const
	{
		if (!m_bEnableCustomProfiler) 
			return;
			
		printf("\n==== Physics Profiler Statistics ====\n");
		printf("Event                      | Count |  Total(us)  |   Avg(us)   |   Min(us)   |   Max(us)   |\n");
		printf("---------------------------+-------+-------------+-------------+-------------+-------------+\n");
		
		for (const auto& pair : m_EventStats)
		{
			const std::string& name = pair.first;
			const EventStats& stats = pair.second;
			double avgTime = stats.count > 0 ? static_cast<double>(stats.totalTime) / stats.count : 0.0;
			
			printf("%-26s | %5u | %11llu | %11.2f | %11llu | %11llu |\n",
				name.c_str(), stats.count, stats.totalTime, avgTime, stats.minTime, stats.maxTime);
		}
		
		printf("==================================\n");
	}

	void resetStatistics()
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		m_EventStats.clear();
		m_CompletedEvents.clear();
		m_DataRecords.clear();
	}

	void setVerboseOutput(bool enable) 
	{ 
		m_bVerboseOutput = enable; 
	}

	void enableCustomProfiler(bool enable) 
	{ 
		m_bEnableCustomProfiler = enable; 
	}

	void enablePVDProfiler(bool enable) 
	{ 
		m_bCallPVDProfilingFunctions = enable && m_bEnablePVD; 
	}

	physx::PxPvd* GetPVD()
	{
		return m_Pvd.get();
	}

	const std::unordered_map<std::string, EventStats>& getEventStats() const
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

private:
	uint64_t getCurrentTimeMicroseconds() const 
	{
		auto now = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(
			now.time_since_epoch()).count();
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
	std::unordered_map<std::string, EventStats> m_EventStats;
};



#if _DEBUG
	#define PHYSICS_CONCAT(X, Y) X##Y
	#define PHYSICS_PROFILE_ZONE(x, y)										\
		PhysicsProfilerScope PHYSICS_CONCAT(_scoped, __LINE__)(g_PhysicsProfiler, x, false, (size_t)y)
	#define PHYSICS_PROFILE_START_CROSSTHREAD(x, y)							\
		if(g_PhysicsProfiler)										\
			g_PhysicsProfiler->zoneStart(x, true, (size_t)y)
	#define PHYSICS_PROFILE_STOP_CROSSTHREAD(x, y)							\
		if(g_PhysicsProfiler)										\
			g_PhysicsProfiler->zoneEnd(NULL, x, true, (size_t)y)
	#define PHYSICS_PROFILE_VALUE(x, y, z)									\
		if(g_PhysicsProfiler)										\
			g_PhysicsProfiler->recordData(x, y, (size_t)z)
	#define PHYSICS_PROFILE_FRAME(x, y)                                                                                                         \
		if(g_PhysicsProfiler)                                                                                                        \
			g_PhysicsProfiler->recordFrame(x, (size_t)y)
#else
	#define PHYSICS_PROFILE_ZONE(x, y)
	#define PHYSICS_PROFILE_START_CROSSTHREAD(x, y)
	#define PHYSICS_PROFILE_STOP_CROSSTHREAD(x, y)
	#define PHYSICS_PROFILE_VALUE(x, y, z)
	#define PHYSICS_PROFILE_FRAME(x, y)
#endif