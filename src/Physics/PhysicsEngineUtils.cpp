#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsEngine.h"

IPhysicsEngine* PhysicsEngineUtils::CreatePhysicsEngine()
{
	return new PhysicsEngine();
}

void PhysicsEngineUtils::DestroyPhysicsEngine()
{
	delete gPhysicsEngine;
	gPhysicsEngine = nullptr;
}