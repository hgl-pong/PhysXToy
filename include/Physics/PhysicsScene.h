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
	PhysicsScene(const PhysicsSceneCreateOptions& options,physx::PxCpuDispatcher* );
	void Release() override;
	void Tick(MathLib::HReal deltaTime) override;
	bool AddPhysicsObject(IPhysicsObject *physicsObject) override;
	void RemovePhysicsObject(IPhysicsObject *physicsObject) override;
	uint32_t GetPhysicsObjectCount() const override;
	uint32_t GetPhysicsRigidDynamicCount() const override;
	uint32_t GetPhysicsRigidStaticCount() const override;
	size_t GetOffset() const override;

private:
	PhysXPtr<physx::PxScene> m_Scene;
	std::unordered_set<PhysicsRigidStatic *> m_RigidStatic;
	std::unordered_set<PhysicsRigidDynamic *> m_RigidDynamic;
};