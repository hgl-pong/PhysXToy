#pragma once
#include <PxPhysicsAPI.h>
#include "Physics/PhysicsMacros.h"

class PhysXErrorCallback : public physx::PxErrorCallback
{
public:
    PhysXErrorCallback()		{}
    virtual	~PhysXErrorCallback()	{}

    virtual void reportError(physx::PxErrorCode::Enum code, const char* message, const char* file, int line)
    {
        PHYSICS_REPORT_ERROR(message, file, line);
    }
};
