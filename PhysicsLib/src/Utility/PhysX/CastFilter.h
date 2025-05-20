#pragma once
#include "Physics/PhysicsCommon.h"
#include "PxPhysicsAPI.h"

enum QueryTriggerInteraction
{
    QueryTriggerInteraction_UseGlobal = 0,
    QueryTriggerInteraction_Ignore = 1,
    QueryTriggerInteraction_Hit = 2
};


class CastFilter : public physx::PxQueryFilterCallback
{
public:
    CastFilter(physx::PxActor* filterActor, bool blocking, unsigned int mask, QueryTriggerInteraction queryTriggerInteraction)
        : m_PostFilterHitType(blocking ? physx::PxQueryHitType::eBLOCK : physx::PxQueryHitType::eTOUCH)
        , m_FilterActor(filterActor)
        , m_Mask(mask)
    {
        switch (queryTriggerInteraction)
        {
            // TODO: implement PhysicsEngineUtils::GetPhysicsEngine()->GetQueriesHitTriggers()
            //case QueryTriggerInteraction_UseGlobal: m_HitTriggers = PhysicsEngineUtils::GetPhysicsEngine()->GetQueriesHitTriggers(); break;
            case QueryTriggerInteraction_Ignore: m_HitTriggers = false; break;
            case QueryTriggerInteraction_Hit: m_HitTriggers = true; break;
            default: ASSERT_MSG(0, "Invalid QueryTriggerInteraction for CastFilter");
        }
    }

    bool IsShapeIgnored(const physx::PxShape* shape, const physx::PxRigidActor* actor) const
    {
        if (actor == m_FilterActor)
            return true;

        bool isTrigger = shape->getFlags() & physx::PxShapeFlag::eTRIGGER_SHAPE;

        if (isTrigger && !m_HitTriggers)
            return true;

        if (actor->is<physx::PxRigidDynamic>())
        {
            IRigidBody* attachedBody = static_cast< IRigidBody*>(actor->userData);

            if (attachedBody && !attachedBody->IsEmpty())
                return true;
        }

        return false;
    }

    virtual physx::PxQueryHitType::Enum preFilter(const physx::PxFilterData& filterData, const physx::PxShape* shape, const physx::PxRigidActor* actor, physx::PxHitFlags& filterFlags)
    {
        if (IsShapeIgnored(shape, actor))
            return physx::PxQueryHitType::eNONE;

        physx::PxFilterData otherData = shape->getQueryFilterData();

        // TODO: implement PhysicsEngineUtils::GetPhysicsEngine()->GetIgnoreCollision(&filterData, &otherData)
        //bool enabledBySelect = !PhysicsEngineUtils::GetPhysicsEngine()->GetIgnoreCollision(&filterData, &otherData);
        bool enabledBySelect = false;

        physx::PxU32 shapeLayer = otherData.word0 & 0xFF;
        physx::PxU32 shapeLayerMask = 1 << shapeLayer;
        bool enabledByMask = m_Mask & shapeLayerMask;

        if (enabledByMask && enabledBySelect)
            return m_PostFilterHitType;

        return physx::PxQueryHitType::eNONE;
    }

    //sschirm: added post filter exclusively for ray casts only. Ignoring shapes that contain the raycast origin, for backwards compatible behavior
    // PT: this is actually now also used for sweeps, to filter out initially overlapping shapes
    virtual physx::PxQueryHitType::Enum postFilter(const physx::PxFilterData& filterData, const physx::PxQueryHit& hit)
    {
        const physx::PxLocationHit& impactHit = static_cast<const physx::PxLocationHit&>(hit);
        if (impactHit.distance > 0.0f)
            return m_PostFilterHitType;

        return physx::PxQueryHitType::eNONE;
    }

    physx::PxQueryHitType::Enum m_PostFilterHitType;
    physx::PxActor* m_FilterActor;
    unsigned int m_Mask;
    bool m_HitTriggers;
};
