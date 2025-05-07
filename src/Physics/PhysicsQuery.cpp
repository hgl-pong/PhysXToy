#include "PhysicsQuery.h"
#include "PhysicsAPI.h"

struct CustomPrunerFilterCallback : physx::PrunerFilterCallback
{
    physx::PxQueryFilterCallback*	m_FilterCB;

    PX_FORCE_INLINE	CustomPrunerFilterCallback(physx::PxQueryFilterCallback* filterCB) : m_FilterCB(filterCB)	{}

    virtual	const physx::PxGeometry*	validatePayload(const physx::PrunerPayload& payload, physx::PxHitFlags& hitFlags)
    {
        physx::PxShape* shape = getShapeFromPayload(payload);

        if(mFilterCB)
        {
            physx::PxRigidActor* actor = getActorFromPayload(payload);
            if(m_FilterCB->preFilter(physx::PxFilterData(), shape, actor, hitFlags)==physx::PxQueryHitType::eNONE)
                return NULL;
        }

        return &shape->getGeometry();
    }
};