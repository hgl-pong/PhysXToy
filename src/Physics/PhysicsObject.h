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
public:	
	void Release()override;
	void Update() override;
	bool IsValid() const override { return m_RigidDynamic != nullptr; };
	bool AddColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry, const MathLib::HTransform3 &localTrans) override;
	void GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>>& geomeries, std::vector<MathLib::HTransform3>* geoLocalPos = nullptr) override
	{
		geomeries = m_ColliderGeometries; 
		if (geoLocalPos)
			*geoLocalPos = m_ColliderLocalPos;
	};
	PhysicsObjectType GetType() const override { return m_Type; };
	size_t GetOffset() const override;
	void SetTransform(const MathLib::HTransform3 &trans) override;
	const MathLib::HTransform3 &GetTransform() const override { return m_Transform; };
	MathLib::HAABBox3D GetLocalBoundingBox() const override { return m_BoundingBox; };
	MathLib::HAABBox3D GetWorldBoundingBox() const override;

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
	bool IsSleeping() const override;

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxRigidDynamic> m_RigidDynamic;
	PhysicsPtr<IPhysicsMaterial>  m_Material;
	std::vector<PhysicsPtr<IColliderGeometry>> m_ColliderGeometries;
	std::vector<MathLib::HTransform3> m_ColliderLocalPos;
	bool m_bIsKinematic;
	MathLib::HReal m_Mass;
	MathLib::HVector3 m_LinearVelocity;
	MathLib::HReal m_AngularDamping;
	MathLib::HVector3 m_AngularVelocity;
	MathLib::HTransform3 m_Transform;
	MathLib::HAABBox3D m_BoundingBox;
};

class PhysicsRigidStatic : public IPhysicsObject
{
public:
	PhysicsRigidStatic(PhysicsPtr < IPhysicsMaterial>& material);
public:	
	void Release()override;
	void Update() override {}
	bool IsValid() const override { return m_RigidStatic != nullptr; };
	void SetTransform(const MathLib::HTransform3 &trans);
	const MathLib::HTransform3 &GetTransform() const override { return m_Transform; };
	bool AddColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry, const MathLib::HTransform3 &localTrans) override;	
	void GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>>& geomeries, std::vector<MathLib::HTransform3>* geoLocalPos=nullptr) override 
	{ 
		geomeries = m_ColliderGeometries; 
		if(geoLocalPos)
			*geoLocalPos = m_ColliderLocalPos;
	};
	PhysicsObjectType GetType() const override { return m_Type; };
	size_t GetOffset() const override;
	MathLib::HAABBox3D GetLocalBoundingBox() const override { return m_BoundingBox; };
	MathLib::HAABBox3D GetWorldBoundingBox() const override;

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxRigidStatic> m_RigidStatic;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	std::vector<PhysicsPtr<IColliderGeometry>> m_ColliderGeometries;
	std::vector<MathLib::HTransform3> m_ColliderLocalPos;
	MathLib::HTransform3 m_Transform;
	MathLib::HAABBox3D m_BoundingBox;
};