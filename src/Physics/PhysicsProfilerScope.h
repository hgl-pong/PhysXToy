#pragma once
#include "PhysicsProfiler.h"

class PhysicsProfilerScope
{
public:
    PhysicsProfilerScope(PhysicsProfiler* profiler, const char* eventName, uint64_t contextId = 0, bool detached = false)
        : m_Profiler(profiler), m_EventName(eventName), m_ContextId(contextId), m_Detached(detached)
    {
        if (m_Profiler)
        {
            m_ProfilerData = m_Profiler->zoneStart(m_EventName, m_Detached, m_ContextId);
        }
    }

    ~PhysicsProfilerScope()
    {
        if (m_Profiler)
        {
            m_Profiler->zoneEnd(m_ProfilerData, m_EventName, m_Detached, m_ContextId);
        }
    }

    PhysicsProfilerScope(const PhysicsProfilerScope&) = delete;
    PhysicsProfilerScope& operator=(const PhysicsProfilerScope&) = delete;

private:
    PhysicsProfiler* m_Profiler;
    const char* m_EventName;
    uint64_t m_ContextId;
    bool m_Detached;
    void* m_ProfilerData;
};

#define PHYSICS_PROFILE_SCOPE(profiler, name, contextId) \
    PhysicsProfilerScope CONCATENATE(profilerScope, __LINE__)((profiler), (name), (contextId))

#define PHYSICS_PROFILE_FUNCTION(profiler, contextId) \
    PhysicsProfilerScope CONCATENATE(profilerScope, __LINE__)((profiler), __FUNCTION__, (contextId))

#define CONCATENATE_IMPL(s1, s2) s1##s2
#define CONCATENATE(s1, s2) CONCATENATE_IMPL(s1, s2) 