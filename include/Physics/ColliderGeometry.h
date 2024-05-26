#pragma once
#include "Physics/PhysicsCommon.h"
namespace physx
{
	class PxShape;
}

class ColliderGeometry :public IColliderGeometry
{
public:

private:
	std::unique_ptr<physx::PxShape> m_Shape;
};