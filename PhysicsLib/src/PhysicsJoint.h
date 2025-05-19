#pragma once
#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsTypes.h"
#include <memory>

namespace physx
{
    class PxJoint;
    class PxFixedJoint;
    class PxDistanceJoint;
    class PxSphericalJoint;
    class PxRevoluteJoint;
    class PxPrismaticJoint;
    class PxD6Joint;
}

class PhysicsJoint : public IPhysicsJoint
{
public:
    PhysicsJoint(JointType type, PhysicsPtr<IPhysicsObject> objectA, PhysicsPtr<IPhysicsObject> objectB, 
                const MathLib::HTransform3& localFrameA, const MathLib::HTransform3& localFrameB, 
                bool collisionEnabled = false);
    virtual ~PhysicsJoint();

    void Release() override;
    JointType GetType() const override;
    void SetBreakForce(const MathLib::HReal& force) override;
    MathLib::HReal GetBreakForce() const override;
    void SetBreakTorque(const MathLib::HReal& torque) override;
    MathLib::HReal GetBreakTorque() const override;
    PhysicsPtr<IPhysicsObject> GetObjectA() const override;
    PhysicsPtr<IPhysicsObject> GetObjectB() const override;
    void SetLocalPose(bool isObjectA, const MathLib::HTransform3& pose) override;
    MathLib::HTransform3 GetLocalPose(bool isObjectA) const override;
    bool IsBroken() const override;
    size_t GetOffset() const override;

    void SetJointLimits(const JointLimitOptions& limitOptions) override;
    JointLimitOptions GetJointLimits() const override;

    physx::PxJoint* GetPxJoint() const;

protected:
    bool CreateHingeJoint();
    bool CreateGearJoint();
    bool CreateRackAndPinionJoint();
    bool CreateChainJoint();
    bool CreatePortalJoint();

    void UpdateJointPose();

protected:
    JointType m_Type;
    PhysicsPtr<IPhysicsObject> m_ObjectA;
    PhysicsPtr<IPhysicsObject> m_ObjectB;
    MathLib::HTransform3 m_LocalFrameA;
    MathLib::HTransform3 m_LocalFrameB;
    bool m_CollisionEnabled;
    bool m_IsBroken;
    MathLib::HReal m_BreakForce;
    MathLib::HReal m_BreakTorque;
    JointLimitOptions m_LimitOptions;

    union {
        physx::PxJoint* m_PxJoint;
        physx::PxFixedJoint* m_PxFixedJoint;
        physx::PxDistanceJoint* m_PxDistanceJoint;
        physx::PxSphericalJoint* m_PxSphericalJoint;
        physx::PxRevoluteJoint* m_PxRevoluteJoint;
        physx::PxPrismaticJoint* m_PxPrismaticJoint;
        physx::PxD6Joint* m_PxD6Joint;
        physx::PxD6Joint* m_PxHingeJoint;
        physx::PxJoint* m_PxGearJoint;
        physx::PxJoint* m_PxRackAndPinionJoint;
        physx::PxJoint* m_PxChainJoint;
        physx::PxJoint* m_PxPortalJoint;
    };
}; 