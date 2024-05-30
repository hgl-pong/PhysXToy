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
	void Release()override;
	void Update();
	void SetKinematic(bool bKinematic);
	bool IsKinematic() const { return m_bIsKinematic; };

public:	
	bool IsValid() const override { return m_RigidDynamic != nullptr; };
	bool AddColliderGeometry(IColliderGeometry *colliderGeometry, const MathLib::HTransform3 &localTrans) override;
	PhysicsObjectType GetType() const override { return m_Type; };
	size_t GetOffset() const override;
	void SetTransform(const MathLib::HTransform3 &trans) override;
public:
	void SetAngularDamping(const MathLib::HReal &damping);
	void SetLinearVelocity(const MathLib::HVector3 &velocity);

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxRigidDynamic> m_RigidDynamic;
	PhysicsPtr<IPhysicsMaterial>  m_Material;
	bool m_bIsKinematic;
	MathLib::HReal m_Mass;
	MathLib::HVector3 m_LinearVelocity;
	MathLib::HReal m_AngularDamping;
	MathLib::HVector3 m_AngularVelocity;
	MathLib::HTransform3 m_Transform;
};

class PhysicsRigidStatic : public IPhysicsObject
{
public:
	PhysicsRigidStatic(IPhysicsMaterial* material);
	void Release()override;
public:	
	bool IsValid() const override { return m_RigidStatic != nullptr; };
	void SetTransform(const MathLib::HTransform3 &trans);
	bool AddColliderGeometry(IColliderGeometry *colliderGeometry, const MathLib::HTransform3 &localTrans) override;
	PhysicsObjectType GetType() const override { return m_Type; };
	size_t GetOffset() const override;

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxRigidStatic> m_RigidStatic;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	MathLib::HTransform3 m_Transform;
};