#pragma once
#include "Physics/PhysicsCommon.h"
#include "PxPhysicsAPI.h"
#include <mutex>
#include <unordered_map>

PHYSICS_INLINE void* PlatformAlignedAlloc(size_t size)
{
	return _aligned_malloc(size, 16);
}

PHYSICS_INLINE void PlatformAlignedFree(void* ptr)
{
	_aligned_free(ptr);
}

class PhysicsProfiler;

class PhysXAllocator : public physx::PxAllocatorCallback
{
public:
	PhysXAllocator() {}
	
    ~PhysXAllocator() 
    {
    #ifdef _DEBUG
        std::lock_guard<std::mutex> lock(m_Mutex);
        if(m_AllocationMap.size() > 0)
        {
            PHYSICS_REPORT_ERROR("Has memory leak in PhysX");
        }
    #endif
    }

	virtual void* allocate(size_t size, const char* typeName, const char* filename, int line)
	{
		void* ptr = PlatformAlignedAlloc(size);
		PX_ASSERT((size_t(ptr) & 15)==0);
#if PX_STOMP_ALLOCATED_MEMORY
		if(ptr != NULL)
		{
			PxMemSet(ptr, PxI32(0xcd), PxU32(size));
		}
#endif
		if (ptr)
		{
			NotifyMemoryAllocated(size);
		}
		
		return ptr;
	}

	virtual void deallocate(void* ptr)
	{
		if (ptr)
		{
			size_t freedSize = 0;
			{
				std::lock_guard<std::mutex> lock(m_Mutex);
				auto it = m_AllocationMap.find(ptr);
				if (it != m_AllocationMap.end())
				{
					freedSize = it->second;
					m_AllocationMap.erase(it);

					NotifyMemoryFreed(freedSize);
				}
			}
			
			PlatformAlignedFree(ptr);
		}
	}
	
	size_t GetActiveAllocationCount() const 
	{ 
#ifdef _DEBUG
		std::lock_guard<std::mutex> lock(m_Mutex); 
		return m_AllocationMap.size(); 
#else
	    return 0;
#endif
	}

private:
	void NotifyMemoryAllocated(size_t size);
	void NotifyMemoryFreed(size_t size);
	
#ifdef _DEBUG
	mutable std::mutex m_Mutex;
	std::unordered_map<void*, size_t> m_AllocationMap;
#endif
};