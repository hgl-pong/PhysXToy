#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
	class PxRigidDynamic;
	class PxRigidStatic;
	class PxHeightField;
}

class RigidDynamic :public IPhysicsObject
{
public:
	void Update();
	void SetKinematic(bool bKinematic);
	bool IsKinematic() const;
	bool IsValid() const;
private:
	physx::PxRigidDynamic* m_pRigidDynamic;
	bool m_bIsKinematic;
	MathLib::HReal m_Mass;
	MathLib::HReal m_LinearVelocity;
	MathLib::HReal m_AngularVelocity;
	MathLib::HTransform3 m_Transform;
};

class RigidStatic :public IPhysicsObject
{
public:
	bool IsValid() const;
private:
	physx::PxRigidStatic* m_pRigidStatic;
	MathLib::HTransform3 m_Transform;
};