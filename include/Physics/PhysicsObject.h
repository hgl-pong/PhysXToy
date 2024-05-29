#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
	class PxRigidDynamic;
	class PxRigidStatic;
	class PxHeightField;
}

class PhysicsRigidDynamic : public IPhysicsObject
{
public:
	PhysicsRigidDynamic(IPhysicsMaterial *material);
	void Update();
	void SetKinematic(bool bKinematic);
	bool IsKinematic() const { return m_bIsKinematic; };
	bool IsValid() const { return m_pRigidDynamic != nullptr; };
	void AddColliderGeometry(IColliderGeometry *colliderGeometry, const MathLib::HTransform3 &localTrans) override;
	PhysicsObjectType GetType() const override { return m_Type; };
	uint32_t GetOffset() const override;
	void SetTransform(const MathLib::HTransform3 &trans);
public:
	void SetAngularDamping(const MathLib::HReal &damping);
	void SetLinearVelocity(const MathLib::HVector3 &velocity);

private:
	PhysicsObjectType m_Type;
	std::unique_ptr<physx::PxRigidDynamic> m_pRigidDynamic;
	std::unique_ptr<IPhysicsMaterial> m_Material;
	bool m_bIsKinematic;
	MathLib::HReal m_Mass;
	MathLib::HReal m_LinearVelocity;
	MathLib::HReal m_AngularDamping;
	MathLib::HVector3 m_AngularVelocity;
	MathLib::HTransform3 m_Transform;
};

class PhysicsRigidStatic : public IPhysicsObject
{
public:
	bool IsValid() const;

private:
	physx::PxRigidStatic *m_pRigidStatic;
	MathLib::HTransform3 m_Transform;
};