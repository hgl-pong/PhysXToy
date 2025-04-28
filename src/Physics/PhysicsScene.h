#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
	class PxPhysics;
	class PxScene;
	class PxCpuDispatcher;
}

class PhysicsEngine;
class PhysicsRigidDynamic;
class PhysicsRigidStatic;

class PhysicsScene : public IPhysicsScene
{
public:
	PhysicsScene(const PhysicsSceneCreateOptions &options, physx::PxCpuDispatcher *cpuDispatch);

	void Release() override;
	void Tick(MathLib::HReal deltaTime) override;
	bool AddPhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject) override;
	void RemovePhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject) override;
	bool AddJoint(PhysicsPtr<IPhysicsJoint> &joint) override;
	void RemoveJoint(PhysicsPtr<IPhysicsJoint> &joint) override;
	bool AddSoftBody(PhysicsPtr<ISoftBody>& softBody);
	void RemoveSoftBody(PhysicsPtr<ISoftBody>& softBody);
	uint32_t GetPhysicsObjectCount() const override;
	uint32_t GetPhysicsRigidDynamicCount() const override;
	uint32_t GetPhysicsRigidStaticCount() const override;
	uint32_t GetJointCount() const override;
	void RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, RaycastHit& hit) override;
	void RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<RaycastHit>& hits) override;
	void SetGravity(const MathLib::HVector3& gravity) override;
	MathLib::HVector3 GetGravity() const override;
	void DebugDraw() override;
	size_t GetOffset() const override;
	void* GetNativeScene() const override;
private:
	PhysXPtr<physx::PxScene> m_Scene;
	PhysicsPtr<IPhysicsObject> m_GroundPlane;
	std::set<PhysicsPtr<IPhysicsObject>> m_PhysicsObjects;
	std::vector<PhysicsPtr<IPhysicsObject>> m_PhysicsRigidDynamics;
	std::vector<PhysicsPtr<IPhysicsObject>> m_PhysicsRigidStatics;
	std::vector<PhysicsPtr<ISoftBody>> m_PhysicsSoftBodies;
	std::vector<PhysicsPtr<IPhysicsObject>> m_PhysicsClothes;
	std::vector<PhysicsPtr<IPhysicsJoint>> m_Joints;
	MathLib::HVector3 m_Gravity;
};