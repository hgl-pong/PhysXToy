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
	bool RemoveColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry) override;
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
	void SetUserData(void* userData) override { m_UserData = userData; }
	void* GetUserData() const override { return m_UserData; }
	void SetCollisionLayer(uint32_t layer) override { m_CollisionLayer = layer; }
	uint32_t GetCollisionLayer() const override { return m_CollisionLayer; }
	void SetCollisionMask(uint32_t mask) override { m_CollisionMask = mask; }
	uint32_t GetCollisionMask() const override { return m_CollisionMask; }

public:
	void SetAngularDamping(const MathLib::HReal &damping)override;
	void SetLinearDamping(const MathLib::HReal &damping)override;
	void SetLinearVelocity(const MathLib::HVector3& velocity)override;
	void SetAngularVelocity(const MathLib::HVector3& velocity)override;
	void SetKinematic(bool bKinematic)override;
	void SetMass(const MathLib::HReal &mass)override;
	void AddForce(const MathLib::HVector3& force, ForceMode mode = ForceMode::FORCE)override;
	void AddTorque(const MathLib::HVector3& torque, ForceMode mode = ForceMode::FORCE)override;
	void AddForceAtLocalPosition(const MathLib::HVector3& force, const MathLib::HVector3& pos, ForceMode mode = ForceMode::FORCE)override;
	void AddForceAtPosition(const MathLib::HVector3& force, const MathLib::HVector3& pos, ForceMode mode = ForceMode::FORCE)override;
	void ClearForce(bool clearVelocity = false)override;
	bool IsKinematic() const override{ return m_bIsKinematic; };
	MathLib::HReal GetMass() const override { return m_Mass; };
	MathLib::HVector3 GetLinearVelocity() const override { return m_LinearVelocity; };
	MathLib::HReal GetAngularDamping() const override { return m_AngularDamping; };
	MathLib::HReal GetLinearDamping() const override { return m_LinearDamping; };
	MathLib::HVector3 GetAngularVelocity() const override { return m_AngularVelocity; };
	MathLib::HMatrix3 GetInertiaTensor() const override;
	bool IsSleeping() const override;
	void SetCenterOfMass(const MathLib::HVector3& centerOfMass) override { m_CenterOfMass = centerOfMass; }
	MathLib::HVector3 GetCenterOfMass() const override { return m_CenterOfMass; }
	void SetGravityEnabled(bool enabled) override { m_GravityEnabled = enabled; }
	bool IsGravityEnabled() const override { return m_GravityEnabled; }
	void SetSleepThreshold(const MathLib::HReal& threshold) override { m_SleepThreshold = threshold; }
	MathLib::HReal GetSleepThreshold() const override { return m_SleepThreshold; }

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
	MathLib::HReal m_LinearDamping;
	MathLib::HVector3 m_AngularVelocity;
	MathLib::HTransform3 m_Transform;
	MathLib::HAABBox3D m_BoundingBox;
	void* m_UserData = nullptr;
	uint32_t m_CollisionLayer = 1;
	uint32_t m_CollisionMask = 0xFFFFFFFF;
	MathLib::HVector3 m_CenterOfMass = MathLib::HVector3(0, 0, 0);
	bool m_GravityEnabled = true;
	MathLib::HReal m_SleepThreshold = 0.05f;
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
	bool RemoveColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry) override;
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
	void SetUserData(void* userData) override { m_UserData = userData; }
	void* GetUserData() const override { return m_UserData; }
	void SetCollisionLayer(uint32_t layer) override { m_CollisionLayer = layer; }
	uint32_t GetCollisionLayer() const override { return m_CollisionLayer; }
	void SetCollisionMask(uint32_t mask) override { m_CollisionMask = mask; }
	uint32_t GetCollisionMask() const override { return m_CollisionMask; }

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxRigidStatic> m_RigidStatic;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	std::vector<PhysicsPtr<IColliderGeometry>> m_ColliderGeometries;
	std::vector<MathLib::HTransform3> m_ColliderLocalPos;
	MathLib::HTransform3 m_Transform;
	MathLib::HAABBox3D m_BoundingBox;
	void* m_UserData = nullptr;
	uint32_t m_CollisionLayer = 1;
	uint32_t m_CollisionMask = 0xFFFFFFFF;
};