#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsEngine.h"
#include "ConvexMeshDecomposer.h"
static PhysicsEngine* gPhysicsEngine = nullptr;
static ConvexMeshDecomposer* gConvexMeshDecomposer = nullptr;
IPhysicsEngine* PhysicsEngineUtils::CreatePhysicsEngine(const PhysicsEngineOptions& options)
{
	_ASSERT(!gPhysicsEngine);
	gPhysicsEngine = new PhysicsEngine(options);
	gConvexMeshDecomposer =new ConvexMeshDecomposer();
	return gPhysicsEngine;
}

void PhysicsEngineUtils::DestroyPhysicsEngine()
{
	if (gPhysicsEngine)
	{
		delete gPhysicsEngine;
		gPhysicsEngine = nullptr;
	}

	if (gConvexMeshDecomposer)
	{
		delete gConvexMeshDecomposer;
		gConvexMeshDecomposer = nullptr;
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

void PhysicsEngineUtils::ConvexDecomposition(const PhysicsMeshData& meshData, const ConvexDecomposeOptions& params, std::vector<PhysicsMeshData>& convexMeshesData)
{
	if (gConvexMeshDecomposer)
		gConvexMeshDecomposer->Decompose(meshData, params, convexMeshesData);
}