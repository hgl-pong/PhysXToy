#pragma once
#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsPointer.h"
namespace physx
{
	class PxPhysics;
	class PxScene;
}
class PhysicsEngine;
class PhysicsRigidDynamic;
class PhysicsRigidStatic;
class PhysicsScene : public IPhysicsScene
{
public:
	PhysicsScene();
	~PhysicsScene();
	void Tick(float deltaTime) override;
	void Init();
	void UnInit();
	bool AddPhysicsObject(IPhysicsObject *physicsObject) override;
	void RemovePhysicsObject(IPhysicsObject *physicsObject) override;
	uint32_t GetPhysicsObjectCount() const override;
	size_t GetOffset() const override;
private:
private:
	friend class PhysicsEngine;
	PhysXPtr<physx::PxScene> m_Scene;
	std::unordered_set<PhysicsRigidStatic *> m_RigidStatic;
	std::unordered_set<PhysicsRigidDynamic *> m_RigidDynamic;
};