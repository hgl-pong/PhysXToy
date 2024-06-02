#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
	class PxRigidDynamic;
	class PxRigidStatic;
	class PxHeightField;
}

class PhysicsRigidDynamic : public IPhysicsObject,virtual public IDynamicObject
{
public:
	PhysicsRigidDynamic(PhysicsPtr < IPhysicsMaterial >&material);

	void Update();

public:	
	void Release()override;
	bool IsValid() const override { return m_RigidDynamic != nullptr; };
	bool AddColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry, const MathLib::HTransform3 &localTrans) override;
	PhysicsObjectType GetType() const override { return m_Type; };
	size_t GetOffset() const override;
	void SetTransform(const MathLib::HTransform3 &trans) override;
	const MathLib::HTransform3 &GetTransform() const override { return m_Transform; };
public:
	void SetAngularDamping(const MathLib::HReal &damping)override;
	void SetLinearVelocity(const MathLib::HVector3& velocity)override;
	void SetAngularVelocity(const MathLib::HVector3& velocity)override;
	void SetKinematic(bool bKinematic)override;
	bool IsKinematic() const override{ return m_bIsKinematic; };
	MathLib::HReal GetMass() const override { return m_Mass; };
	MathLib::HVector3 GetLinearVelocity() const override { return m_LinearVelocity; };
	MathLib::HReal GetAngularDamping() const override { return m_AngularDamping; };
	MathLib::HVector3 GetAngularVelocity() const override { return m_AngularVelocity; };

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
	PhysicsRigidStatic(PhysicsPtr < IPhysicsMaterial>& material);
	void Release()override;
public:	
	bool IsValid() const override { return m_RigidStatic != nullptr; };
	void SetTransform(const MathLib::HTransform3 &trans);
	const MathLib::HTransform3 &GetTransform() const override { return m_Transform; };
	bool AddColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry, const MathLib::HTransform3 &localTrans) override;
	PhysicsObjectType GetType() const override { return m_Type; };
	size_t GetOffset() const override;

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxRigidStatic> m_RigidStatic;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	MathLib::HTransform3 m_Transform;
};