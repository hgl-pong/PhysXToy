#pragma once
#include "PxPhysicsAPI.h"
#include "pvd/PxPvdTransport.h"
#include "Physics/PhysicsTypes.h"

#define PHYSX_PVD_HOST "127.0.0.1"
class PhysicsProfiler : public physx::PxProfilerCallback
{
public:
	PhysicsProfiler(bool bEnablePVD)
	{
		if (bEnablePVD)
		{
			m_Pvd = make_physx_ptr(PxCreatePvd(PxGetFoundation()));
			physx::PxPvdTransport *transport = PxDefaultPvdSocketTransportCreate(PHYSX_PVD_HOST, 5425, 10);
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

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId)
	{
		// Option 1: add your own profiling code here (before calling the PVD function).
		// If you call the PVD profiling function below, adding your own profiling code here
		// means it will capture the cost of the PVD zoneStart function.

		// NB: we don't have an actual profiler implementation in the snippet so we just call printf instead
		printf("start: %s\n", eventName);

		// Optional: call the PVD function if you want to see the profiling results in PVD.
		void* profilerData = m_bCallPVDProfilingFunctions ? m_Pvd->zoneStart(eventName, detached, contextId) : NULL;

		// Option 2: add your own profiling code here (after calling the PVD function).
		// If you call the PVD profiling function above, adding your own profiling code here
		// means its cost will be captured by the PVD profiler.

		return profilerData;
	}

	virtual void zoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId)
	{
		// Option 2: add your own profiling code here (before calling the PVD function).
		// If you call the PVD profiling function below, adding your own profiling code here
		// means its cost will be captured by the PVD profiler.

		// Optional: call the PVD function if you want to see the profiling results in PVD.
		if (m_bCallPVDProfilingFunctions)
			m_Pvd->zoneEnd(profilerData, eventName, detached, contextId);

		// Option 1: add your own profiling code here (after calling the PVD function).
		// If you call the PVD profiling function above, adding your own profiling code here
		// means it will capture the cost of the PVD zoneEnd function.

		// NB: we don't have an actual profiler implementation in the snippet so we just call printf instead
		printf("end: %s\n", eventName);
	}

	virtual void recordData(int32_t value, const char* valueName, uint64_t contextId)
	{
		printf("data: %s (context ID %llu) = %d\n", valueName, (unsigned long long)contextId, value);
	}

	virtual void recordData(float value, const char* valueName, uint64_t contextId)
	{
		printf("data: %s (context ID %llu) = %f\n", valueName, (unsigned long long)contextId, (double)value);
	}

	virtual void recordFrame(const char* name, uint64_t contextId)
	{
		printf("frame: %s (context ID %llu)\n", name, (unsigned long long)contextId);
	}

	physx::PxPvd* GetPVD()
	{
		return m_Pvd.get();
	}
private:
	bool m_bCallPVDProfilingFunctions = true;
	PhysXPtr<physx::PxPvd> m_Pvd;
};