#pragma once
#include "Physics/PhysicsCommon.h"
namespace physx
{
	class PxPhysics;
	class PxScene;
}
class PhysicsEngine;
class PhysicsScene : public IPhysicsScene
{
public:
	PhysicsScene();
	~PhysicsScene();
	void Tick(float deltaTime) override;
	void Init() ;
	void UnInit() ;
	bool AddPhysicsObject(IPhysicsObject *physicsObject) override;
	void RemovePhysicsObject(IPhysicsObject *physicsObject) override;
	uint32_t GetPhysicsObjectCount() const override;

private:
private:
	friend class PhysicsEngine;
	std::unique_ptr<physx::PxScene> m_Scene;
	std::unordered_set<IPhysicsObject *> m_PhysicsObjects;
};