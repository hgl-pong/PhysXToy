#pragma once
#include "Physics/PhysicsCommon.h"
#include "Utility/PhysxUtils.h"
#include <memory>

namespace physx
{
	class PxArticulationReducedCoordinate;
	class PxArticulationLink;
	class PxArticulationJointReducedCoordinate;
	class PxArticulationCache;
}

class PhysicsArticulation : public IArticulation, public std::enable_shared_from_this<PhysicsArticulation>
{
public:
	PhysicsArticulation(const ArticulationCreateOptions& options);
	virtual ~PhysicsArticulation();

	// IArticulation interface
	void Release() override;
	
	// Basic properties
	PhysicsPtr<IPhysicsScene> GetScene() const override;
	bool IsFixedBase() const override;
	void SetSolverIterationCounts(uint32_t minPositionIters, uint32_t minVelocityIters = 1) override;
	void GetSolverIterationCounts(uint32_t& minPositionIters, uint32_t& minVelocityIters) const override;
	
	// Link management
	PhysicsPtr<IArticulationLink> CreateLink(PhysicsPtr<IArticulationLink> parent, const ArticulationLinkCreateOptions& options) override;
	uint32_t GetNbLinks() const override;
	uint32_t GetLinks(std::vector<PhysicsPtr<IArticulationLink>>& links) const override;
	PhysicsPtr<IArticulationLink> GetRootLink() const override;
	
	// Joint management
	uint32_t GetDofs() const override;
	void UpdateKinematic() override;
	
	// Cache management
	PhysicsPtr<IArticulationCache> CreateCache() override;
	void ApplyCache(PhysicsPtr<IArticulationCache> cache, uint32_t flag) override;
	void CopyInternalStateToCache(PhysicsPtr<IArticulationCache> cache, uint32_t flag) const override;
	
	// Projection and separation settings
	void SetMaxProjectionIterations(uint32_t iterations) override;
	uint32_t GetMaxProjectionIterations() const override;
	void SetSeparationTolerance(MathLib::HReal tolerance) override;
	MathLib::HReal GetSeparationTolerance() const override;
	
	// Drive iteration settings
	void SetInternalDriveIterations(uint32_t iterations) override;
	uint32_t GetInternalDriveIterations() const override;
	void SetExternalDriveIterations(uint32_t iterations) override;
	uint32_t GetExternalDriveIterations() const override;
	
	// Sleep and wake
	bool IsSleeping() const override;
	void SetSleepThreshold(MathLib::HReal threshold) override;
	MathLib::HReal GetSleepThreshold() const override;
	void SetStabilizationThreshold(MathLib::HReal threshold) override;
	MathLib::HReal GetStabilizationThreshold() const override;
	void SetWakeCounter(MathLib::HReal wakeCounterValue) override;
	MathLib::HReal GetWakeCounter() const override;
	void WakeUp() override;
	void PutToSleep() override;
	
	// Advanced computation functions
	void CommonInit() override;
	void ComputeGeneralizedGravityForce(PhysicsPtr<IArticulationCache> cache) override;
	void ComputeCoriolisAndCentrifugalForce(PhysicsPtr<IArticulationCache> cache) override;
	void ComputeGeneralizedExternalForce(PhysicsPtr<IArticulationCache> cache) override;
	void ComputeJointAcceleration(PhysicsPtr<IArticulationCache> cache) override;
	void ComputeJointForce(PhysicsPtr<IArticulationCache> cache) override;
	void ComputeGeneralizedMassMatrix(PhysicsPtr<IArticulationCache> cache) override;
	void ComputeDenseJacobian(PhysicsPtr<IArticulationCache> cache, uint32_t& nRows, uint32_t& nCols) override;
	
	// Name and bounds
	void SetName(const char* name) override;
	const char* GetName() const override;
	MathLib::HAABBox3D GetWorldBounds(MathLib::HReal inflation = 1.01f) const override;
	
	// User data and internal index
	void SetUserData(void* userData) override;
	void* GetUserData() const override;
	uint32_t GetInternalActorIndex() const override;
	
	size_t GetOffset() const override;
	void* GetNativeArticulation() const override;

	// Internal methods
	physx::PxArticulationReducedCoordinate* GetPxArticulation() const { return m_PxArticulation; }

private:
	physx::PxArticulationReducedCoordinate* m_PxArticulation;
	PhysicsPtr<IPhysicsScene> m_Scene;
	std::vector<PhysicsPtr<IArticulationLink>> m_Links;
	PhysicsPtr<IArticulationLink> m_RootLink;
	ArticulationCreateOptions m_Options;
	std::string m_Name;
	void* m_UserData;
};

class PhysicsArticulationLink : public IArticulationLink
{
public:
	PhysicsArticulationLink(PhysicsArticulation* articulation, PhysicsPtr<IArticulationLink> parent, 
							const ArticulationLinkCreateOptions& options);
	virtual ~PhysicsArticulationLink();

	// IPhysicsObject interface (inherited through IArticulationLink)
	void Release() override;
	void Update() override;
	bool AddColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry, const MathLib::HTransform3 &localTrans) override;
	bool RemoveColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry) override;
	void GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>> &geometries, std::vector<MathLib::HTransform3> *geoLocalPos = nullptr) override;
	PhysicsObjectType GetType() const override;
	size_t GetOffset() const override;
	void SetTransform(const MathLib::HTransform3 &trans) override;
	const MathLib::HTransform3 &GetTransform() const override;
	bool IsValid() const override;
	MathLib::HAABBox3D GetLocalBoundingBox() const override;
	MathLib::HAABBox3D GetWorldBoundingBox() const override;
	void SetUserData(void* userData) override;
	void* GetUserData() const override;
	void SetCollisionLayer(uint32_t layer) override;
	uint32_t GetCollisionLayer() const override;
	void SetCollisionMask(uint32_t mask) override;
	uint32_t GetCollisionMask() const override;
	void SetCollisionCallback(ICollisionCallback* callback) override;
	ICollisionCallback* GetCollisionCallback() const override;

	// IArticulationLink interface
	IArticulation* GetArticulation() const override;
	IArticulationJoint* GetInboundJoint() const override;
	uint32_t GetInboundJointDof() const override;
	uint32_t GetLinkIndex() const override;
	
	// Child link management
	uint32_t GetNbChildren() const override;
	uint32_t GetChildren(std::vector<PhysicsPtr<IArticulationLink>>& children) const override;
	
	// Mass properties
	void SetMass(MathLib::HReal mass) override;
	MathLib::HReal GetMass() const override;
	MathLib::HReal GetInvMass() const override;
	void SetCMassLocalPose(const MathLib::HTransform3& pose) override;
	MathLib::HTransform3 GetCMassLocalPose() const override;
	void SetMassSpaceInertiaTensor(const MathLib::HVector3& inertia) override;
	MathLib::HVector3 GetMassSpaceInertiaTensor() const override;
	MathLib::HVector3 GetMassSpaceInvInertiaTensor() const override;
	
	// Damping
	void SetLinearDamping(MathLib::HReal damping) override;
	MathLib::HReal GetLinearDamping() const override;
	void SetAngularDamping(MathLib::HReal damping) override;
	MathLib::HReal GetAngularDamping() const override;
	
	// Velocity limits
	void SetMaxLinearVelocity(MathLib::HReal maxVel) override;
	MathLib::HReal GetMaxLinearVelocity() const override;
	void SetMaxAngularVelocity(MathLib::HReal maxVel) override;
	MathLib::HReal GetMaxAngularVelocity() const override;
	
	// Velocity and acceleration
	MathLib::HVector3 GetLinearVelocity() const override;
	MathLib::HVector3 GetAngularVelocity() const override;
	MathLib::HVector3 GetLinearAcceleration() const override;
	MathLib::HVector3 GetAngularAcceleration() const override;
	
	// Force application
	void AddForce(const MathLib::HVector3& force, ForceMode mode = ForceMode::FORCE, bool autowake = true) override;
	void AddTorque(const MathLib::HVector3& torque, ForceMode mode = ForceMode::FORCE, bool autowake = true) override;
	void ClearForce(ForceMode mode = ForceMode::FORCE) override;
	void ClearTorque(ForceMode mode = ForceMode::FORCE) override;
	void SetForceAndTorque(const MathLib::HVector3& force, const MathLib::HVector3& torque, ForceMode mode = ForceMode::FORCE) override;
	
	// CCD and physical properties
	void SetMinCCDAdvanceCoefficient(MathLib::HReal coefficient) override;
	MathLib::HReal GetMinCCDAdvanceCoefficient() const override;
	void SetMaxDepenetrationVelocity(MathLib::HReal maxVel) override;
	MathLib::HReal GetMaxDepenetrationVelocity() const override;
	void SetMaxContactImpulse(MathLib::HReal maxImpulse) override;
	MathLib::HReal GetMaxContactImpulse() const override;
	void SetContactSlopCoefficient(MathLib::HReal coefficient) override;
	MathLib::HReal GetContactSlopCoefficient() const override;
	
	// CFM (Constraint Force Mixing)
	void SetCfmScale(MathLib::HReal cfm) override;
	MathLib::HReal GetCfmScale() const override;
	
	void* GetNativeLink() const override;

	// Internal methods
	physx::PxArticulationLink* GetPxLink() const { return m_PxLink; }
	void SetPxLink(physx::PxArticulationLink* pxLink) { m_PxLink = pxLink; }
	void AddChild(PhysicsPtr<IArticulationLink> child);
	void SetInboundJoint(PhysicsPtr<IArticulationJoint> joint) { m_InboundJoint = joint; }

private:
	physx::PxArticulationLink* m_PxLink;
	PhysicsArticulation* m_Articulation;
	PhysicsPtr<IArticulationLink> m_Parent;
	std::vector<PhysicsPtr<IArticulationLink>> m_Children;
	PhysicsPtr<IArticulationJoint> m_InboundJoint;
	ArticulationLinkCreateOptions m_Options;
	mutable MathLib::HTransform3 m_Transform;
	void* m_UserData;
	uint32_t m_CollisionLayer;
	uint32_t m_CollisionMask;
	ICollisionCallback* m_CollisionCallback;
	std::vector<PhysicsPtr<IColliderGeometry>> m_ColliderGeometries;
	std::vector<MathLib::HTransform3> m_ColliderLocalTransforms;
};

class PhysicsArticulationJoint : public IArticulationJoint
{
public:
	PhysicsArticulationJoint(PhysicsPtr<IArticulationLink> link, const ArticulationJointCreateOptions& options);
	virtual ~PhysicsArticulationJoint();

	// Basic properties
	ArticulationJointType GetJointType() const override;
	void SetParentPose(const MathLib::HTransform3& pose) override;
	MathLib::HTransform3 GetParentPose() const override;
	void SetChildPose(const MathLib::HTransform3& pose) override;
	MathLib::HTransform3 GetChildPose() const override;
	
	// Motion configuration
	void SetMotion(ArticulationAxis axis, ArticulationMotion motion) override;
	ArticulationMotion GetMotion(ArticulationAxis axis) const override;
	uint32_t GetDofCount() const override;
	
	// Limit settings
	void SetLimit(ArticulationAxis axis, const ArticulationLimit& limit) override;
	ArticulationLimit GetLimit(ArticulationAxis axis) const override;
	
	// Drive settings
	void SetDrive(ArticulationAxis axis, const ArticulationDrive& drive) override;
	ArticulationDrive GetDrive(ArticulationAxis axis) const override;
	void SetDriveTarget(ArticulationAxis axis, MathLib::HReal target) override;
	MathLib::HReal GetDriveTarget(ArticulationAxis axis) const override;
	void SetDriveVelocity(ArticulationAxis axis, MathLib::HReal velocity) override;
	MathLib::HReal GetDriveVelocity(ArticulationAxis axis) const override;
	
	// Joint position and velocity
	void SetJointPosition(ArticulationAxis axis, MathLib::HReal position) override;
	MathLib::HReal GetJointPosition(ArticulationAxis axis) const override;
	void SetJointVelocity(ArticulationAxis axis, MathLib::HReal velocity) override;
	MathLib::HReal GetJointVelocity(ArticulationAxis axis) const override;
	
	// Physical properties
	void SetFrictionCoefficient(MathLib::HReal coefficient) override;
	MathLib::HReal GetFrictionCoefficient() const override;
	void SetMaxJointVelocity(MathLib::HReal maxVelocity) override;
	MathLib::HReal GetMaxJointVelocity() const override;
	
	size_t GetOffset() const override;

	// Internal methods
	physx::PxArticulationJointReducedCoordinate* GetPxJoint() const { return m_PxJoint; }

private:
	physx::PxArticulationJointReducedCoordinate* m_PxJoint;
	PhysicsPtr<IArticulationLink> m_Link;
	ArticulationJointCreateOptions m_Options;
};

class PhysicsArticulationCache : public IArticulationCache
{
public:
	PhysicsArticulationCache(PhysicsPtr<IArticulation> articulation);
	virtual ~PhysicsArticulationCache();

	void Release() override;
	
	// Data access interface
	MathLib::HReal* GetJointPositions() override;
	const MathLib::HReal* GetJointPositions() const override;
	MathLib::HReal* GetJointVelocities() override;
	const MathLib::HReal* GetJointVelocities() const override;
	MathLib::HReal* GetJointAccelerations() override;
	const MathLib::HReal* GetJointAccelerations() const override;
	MathLib::HReal* GetJointForces() override;
	const MathLib::HReal* GetJointForces() const override;
	
	ArticulationSpatialVelocity* GetLinkVelocities() override;
	const ArticulationSpatialVelocity* GetLinkVelocities() const override;
	ArticulationSpatialVelocity* GetLinkAccelerations() override;
	const ArticulationSpatialVelocity* GetLinkAccelerations() const override;
	
	ArticulationRootLinkData* GetRootLinkData() override;
	const ArticulationRootLinkData* GetRootLinkData() const override;
	
	ArticulationSpatialForce* GetExternalForces() override;
	const ArticulationSpatialForce* GetExternalForces() const override;
	
	// Advanced computation data
	MathLib::HReal* GetMassMatrix() override;
	const MathLib::HReal* GetMassMatrix() const override;
	MathLib::HReal* GetJacobian() override;
	const MathLib::HReal* GetJacobian() const override;

	// Internal methods
	physx::PxArticulationCache* GetPxCache() const { return m_PxCache; }

private:
	physx::PxArticulationCache* m_PxCache;
	PhysicsPtr<IArticulation> m_Articulation;
}; 