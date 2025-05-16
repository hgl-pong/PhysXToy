#pragma once
#include "Physics/Joint/PhysicsJoint.h"
#include "Physics/IRevoluteJoint.h"

class RevoluteJoint : public PhysicsJoint, public IRevoluteJoint
{
public:
    RevoluteJoint(const JointCreateOptions& options);
    virtual ~RevoluteJoint();

    // IRevoluteJoint interface implementation
    virtual void SetDriveVelocity(MathLib::HReal velocity) override;
    virtual MathLib::HReal GetDriveVelocity() const override;
    virtual void EnableDrive(bool enable) override;
    virtual bool IsDriveEnabled() const override;
    virtual void SetDriveForceLimit(MathLib::HReal forceLimit) override;
    virtual MathLib::HReal GetDriveForceLimit() const override;
    virtual void SetLimit(MathLib::HReal lowerLimit, MathLib::HReal upperLimit) override;
    virtual void GetLimit(MathLib::HReal& lowerLimit, MathLib::HReal& upperLimit) const override;
    virtual void EnableLimit(bool enable) override;
    virtual bool IsLimitEnabled() const override;
    virtual MathLib::HReal GetAngle() const override;
    virtual MathLib::HReal GetVelocity() const override;

private:
    physx::PxRevoluteJoint* m_PxJoint;
}; 