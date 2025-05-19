#pragma once
#include "Physics/PhysicsCommon.h"
#include "Utility/PhysXUtils.h"
#include <vector>
#include "PxScene.h"

typedef std::vector<PhysicsRaycastHit> PhysicsRaycastHits;
static const int kRaycastMaxHits = 128;


class PhysicsRaycastCallback : public physx::PxRaycastCallback
{
private:
    PhysicsRaycastHits m_RaycastHits;
    std::vector<physx::PxRaycastHit> m_Buffer;

public:

    PhysicsRaycastCallback(int maxHits = kRaycastMaxHits)
        : physx::PxRaycastCallback(NULL, 0)
    {
        m_RaycastHits.reserve(maxHits);
        m_Buffer.resize(maxHits);
        touches = &m_Buffer[0];
        maxNbTouches = m_Buffer.capacity();
    }

    virtual physx::PxAgain  processTouches(const physx::PxRaycastHit *buffer, physx::PxU32 nbHits)
    {
        m_RaycastHits.reserve(m_RaycastHits.size() + nbHits);
        PhysicsRaycastHit thisHit;

        for (size_t i = 0; i < nbHits; ++i)
        {
            ConvertUtils::FromPx(buffer[i], thisHit);
            m_RaycastHits.push_back(thisHit);
        }
        return true;
    }

    virtual void finalizeQuery()
    {
        if (this->hasBlock)
        {
            PhysicsRaycastHit blockingHit;
            ConvertUtils::FromPx(this->block, blockingHit);
            m_RaycastHits.push_back(blockingHit);
        }
    }

    const PhysicsRaycastHits& GetResults() const
    {
        return m_RaycastHits;
    }

    const void ClearBuffers()
    {
        m_RaycastHits.clear();
        m_Buffer.clear();
    }
};

class PhysicsSweepCallback : public physx::PxSweepCallback
{
private:
    PhysicsRaycastHits m_RaycastHits;
    std::vector<physx::PxSweepHit> m_Buffer;

public:

    PhysicsSweepCallback(int maxHits = kRaycastMaxHits)
        : physx::PxSweepCallback(NULL, 0)
    {
        m_RaycastHits.reserve(maxHits);
        m_Buffer.resize(maxHits);
        touches = &m_Buffer[0];
        maxNbTouches = m_Buffer.capacity();
    }

    virtual physx::PxAgain  processTouches(const physx::PxSweepHit *buffer, physx::PxU32 nbHits)
    {
        m_RaycastHits.reserve(m_RaycastHits.size() + nbHits);
        PhysicsRaycastHit thisHit;
        for (size_t i = 0; i < nbHits; ++i)
        {
            ConvertUtils::FromPx(buffer[i], thisHit);
            m_RaycastHits.push_back(thisHit);
        }

        return true;
    }

    virtual void finalizeQuery()
    {
        if (this->hasBlock)
        {
            PhysicsRaycastHit blockingHit;
            ConvertUtils::FromPx(this->block, blockingHit);
            m_RaycastHits.push_back(blockingHit);
        }
    }

    const PhysicsRaycastHits& GetResults() const
    {
        return m_RaycastHits;
    }

    const void ClearBuffers()
    {
        m_RaycastHits.clear();
        m_Buffer.clear();
    }
};
