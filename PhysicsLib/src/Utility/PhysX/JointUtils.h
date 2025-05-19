#pragma once
#include "PxPhysicsAPI.h"
#include "Utility/PhysXUtils.h"
#include "Utility/PhysicsUtils.h"

namespace JointUtils
{
    /**
     * Create Spherical Joint
     * @param actor0 First rigid body
     * @param localFrame0 Local frame of the first rigid body
     * @param actor1 Second rigid body
     * @param localFrame1 Local frame of the second rigid body
     * @param limit Limits of the joint
     * @return Pointer to the created spherical joint, nullptr if creation fails
     */
    inline physx::PxSphericalJoint* CreateSphericalJoint(
        physx::PxRigidActor* actor0,
        const physx::PxTransform& localFrame0,
        physx::PxRigidActor* actor1,
        const physx::PxTransform& localFrame1,
        const PhysicsLimits& limit)
    {
        if (!actor0 || !actor1)
            return nullptr;
        physx::PxSphericalJoint* joint = physx::PxSphericalJointCreate(PxGetPhysics(), actor0, localFrame0, actor1, localFrame1);
        if(joint)
        {
            const bool ValidLimits = IsSphericalLimitEnabled(limit);
            if(ValidLimits)
            {
                const physx::PxJointLimitCone limits(limit.m_MinValue, limit.m_MaxValue);
                joint->setLimitCone(limits);
            }
            joint->setSphericalJointFlag(physx::PxSphericalJointFlag::eLIMIT_ENABLED, ValidLimits);
        }
        return joint;
    }
    
    /**
     * Create Revolute Joint 2 (Alternative implementation of RevoluteJoint)
     * @param actor0 First rigid body
     * @param localFrame0 Local frame of the first rigid body
     * @param actor1 Second rigid body
     * @param localFrame1 Local frame of the second rigid body
     * @return Pointer to the created revolute joint, nullptr if creation fails
     */
    inline physx::PxRevoluteJoint* CreateRevoluteJoint2(
        physx::PxRigidActor* actor0,
        const physx::PxTransform& localFrame0,
        physx::PxRigidActor* actor1,
        const physx::PxTransform& localFrame1)
    {
        if (!actor0 || !actor1)
            return nullptr;
            
        physx::PxRevoluteJoint* joint = physx::PxRevoluteJointCreate(PxGetPhysics(), actor0, localFrame0, actor1, localFrame1);
        
        // Additional configuration, different from CreateRevoluteJoint
        if (joint)
        {
            // Default does not limit rotation
            joint->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, false);
            // Allow driving
            joint->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eDRIVE_ENABLED, true);
        }
        
        return joint;
    }
    
    /**
     * Create Fixed Joint
     * @param actor0 First rigid body
     * @param localFrame0 Local frame of the first rigid body
     * @param actor1 Second rigid body
     * @param localFrame1 Local frame of the second rigid body
     * @return Pointer to the created fixed joint, nullptr if creation fails
     */
    inline physx::PxFixedJoint* CreateFixedJoint(
        physx::PxRigidActor* actor0,
        const physx::PxTransform& localFrame0,
        physx::PxRigidActor* actor1,
        const physx::PxTransform& localFrame1)
    {
        if (!actor0 || !actor1)
            return nullptr;
            
        return physx::PxFixedJointCreate(PxGetPhysics(), actor0, localFrame0, actor1, localFrame1);
    }
    
    /**
     * Create Prismatic Joint
     * @param actor0 First rigid body
     * @param localFrame0 Local frame of the first rigid body
     * @param actor1 Second rigid body
     * @param localFrame1 Local frame of the second rigid body
     * @param limit Limits of the joint
     * @param spring Spring of the joint
     * @return Pointer to the created prismatic joint, nullptr if creation fails
     */
    inline physx::PxPrismaticJoint* CreatePrismaticJoint(
        physx::PxRigidActor* actor0,
        const physx::PxTransform& localFrame0,
        physx::PxRigidActor* actor1,
        const physx::PxTransform& localFrame1,
        const PhysicsLimits& limit,
        const PhysicsSpring& spring)
    {
        if (!actor0 || !actor1)
            return nullptr;
        
        physx::PxPrismaticJoint* joint = physx::PxPrismaticJointCreate(PxGetPhysics(), actor0, localFrame0, actor1, localFrame1);
        if(joint)
        {
            float MinLimit = 0.0f;	// See above note about limits
            float MaxLimit = 0.0f;
            const bool ValidLimits = IsHingeLimitEnabled(limit);
            if(ValidLimits)
            {
                MinLimit = limit.m_MinValue;
                MaxLimit = limit.m_MaxValue;
            }

            // We setup the limits even when invalid, to preserve other params (spring, etc)
            physx::PxJointLinearLimitPair Limits(MinLimit, MaxLimit, physx::PxSpring(spring.m_Stiffness, spring.m_Damping));
            joint->setLimit(Limits);
            joint->setPrismaticJointFlag(physx::PxPrismaticJointFlag::eLIMIT_ENABLED, ValidLimits);

        }
        return joint;
    }
    
    /**
     * Create Distance Joint
     * @param actor0 First rigid body
     * @param localFrame0 Local frame of the first rigid body
     * @param actor1 Second rigid body
     * @param localFrame1 Local frame of the second rigid body
     * @param limit Limits of the joint
     * @return Pointer to the created distance joint, nullptr if creation fails
     */
    inline physx::PxDistanceJoint* CreateDistanceJoint(
        physx::PxRigidActor* actor0,
        const physx::PxTransform& localFrame0,
        physx::PxRigidActor* actor1,
        const physx::PxTransform& localFrame1,
        const PhysicsLimits& limit)
    {
        if (!actor0 || !actor1)
            return nullptr;
        
        physx::PxDistanceJoint* joint = physx::PxDistanceJointCreate(PxGetPhysics(), actor0, localFrame0, actor1, localFrame1);
        if (joint)
        {
            if(IsMaxDistanceLimitEnabled(limit))
            {
                joint->setMaxDistance(limit.m_MaxValue);
                joint->setDistanceJointFlag(physx::PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
            }
            else joint->setDistanceJointFlag(physx::PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, false);

            if(IsMinDistanceLimitEnabled(limit))
            {
                joint->setMinDistance(limit.m_MinValue);
                joint->setDistanceJointFlag(physx::PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
            }
            else joint->setDistanceJointFlag(physx::PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, false);
        }
        return joint;
    }
}
