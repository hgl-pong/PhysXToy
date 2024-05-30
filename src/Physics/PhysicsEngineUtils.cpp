#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsEngine.h"

IPhysicsEngine* PhysicsEngineUtils::CreatePhysicsEngine()
{
	gPhysicsEngine = new PhysicsEngine();
	return gPhysicsEngine;
}

void PhysicsEngineUtils::DestroyPhysicsEngine()
{
	if (gPhysicsEngine)
	{
		gPhysicsEngine->UnInit();
		delete gPhysicsEngine;
		gPhysicsEngine = nullptr;
	}
}

IPhysicsEngine* PhysicsEngineUtils::GetPhysicsEngine()
{
	return gPhysicsEngine;
}

IPhysicsObject* PhysicsEngineUtils::CreateObject(const PhysicsObjectCreateOptions& options) 
{
	return gPhysicsEngine->CreateObject(options);
}
IPhysicsMaterial* PhysicsEngineUtils::CreateMaterial(const PhysicsMaterialCreateOptions& options) 
{
	return gPhysicsEngine->CreateMaterial(options);
}
IPhysicsScene* PhysicsEngineUtils::CreateScene(const PhysicsSceneCreateOptions& options) 
{
	return gPhysicsEngine->CreateScene(options);
}
IColliderGeometry* PhysicsEngineUtils::CreateColliderGeometry(const CollisionGeometryCreateOptions& options) 
{
	return gPhysicsEngine->CreateColliderGeometry(options);
}
