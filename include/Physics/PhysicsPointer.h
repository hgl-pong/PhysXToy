#pragma once
#include <memory>
#include "foundation/PxAllocator.h"
template <typename T>
struct PhysXDeleter 
{
    void operator()(T* ptr) const {
        PX_RELEASE(ptr);
    }
};

template <typename T>
using PhysXPtr = std::unique_ptr<T, PhysXDeleter<T>>;

template <typename T>
PhysXPtr<T> make_physx_ptr(T* ptr) {
    return PhysXPtr<T>(ptr);
}
