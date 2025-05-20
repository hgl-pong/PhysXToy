#include "Allocator.h"
#include "../../PhysicsProfiler.h"

void PhysXAllocator::NotifyMemoryAllocated(size_t size)
{
    PHYSICS_PROFILE_ADD_MEMORY_USAGE(size);
        
    PHYSICS_PROFILE_VALUE("PhysX.MemoryAllocated", static_cast<int32_t>(size / 1024), 0);
    PHYSICS_PROFILE_VALUE("PhysX.AllocationCount", static_cast<int32_t>(m_AllocationMap.size()), 0);
}

void PhysXAllocator::NotifyMemoryFreed(size_t size)
{
    PHYSICS_PROFILE_SUBTRACT_MEMORY_USAGE(size);
        
    PHYSICS_PROFILE_VALUE("PhysX.MemoryFreed", static_cast<int32_t>(size / 1024), 0);
    PHYSICS_PROFILE_VALUE("PhysX.AllocationCount", static_cast<int32_t>(m_AllocationMap.size()), 0);
} 