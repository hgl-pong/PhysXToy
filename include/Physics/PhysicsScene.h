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
	void Init() override;
	void UnInit() override;

private:
private:
	friend class PhysicsEngine;
	std::unique_ptr<physx::PxScene> m_Scene;
};