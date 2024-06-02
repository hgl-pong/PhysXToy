#include "Physics/PhysicsCommon.h"
#include "PhysicsEngine.h"
#include "ConvexMeshDecomposer.h"
#include "Utility/PhysicsConvexUtils.h"
static PhysicsEngine* gPhysicsEngine = nullptr;
static ConvexMeshDecomposer* gConvexMeshDecomposer = nullptr;
IPhysicsEngine* PhysicsEngineUtils::CreatePhysicsEngine(const PhysicsEngineOptions& options, const bool createConvexDecomposer)
{
	_ASSERT(!gPhysicsEngine);
	gPhysicsEngine = new PhysicsEngine(options);
	if(createConvexDecomposer)
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

PhysicsPtr < IPhysicsObject> PhysicsEngineUtils::CreateObject(const PhysicsObjectCreateOptions& options)
{
	return gPhysicsEngine->CreateObject(options);
}
PhysicsPtr < IPhysicsMaterial> PhysicsEngineUtils::CreateMaterial(const PhysicsMaterialCreateOptions& options)
{
	return gPhysicsEngine->CreateMaterial(options);
}
PhysicsPtr < IPhysicsScene> PhysicsEngineUtils::CreateScene(const PhysicsSceneCreateOptions& options)
{
	return gPhysicsEngine->CreateScene(options);
}
PhysicsPtr < IColliderGeometry> PhysicsEngineUtils::CreateColliderGeometry(const CollisionGeometryCreateOptions& options)
{
	return gPhysicsEngine->CreateColliderGeometry(options);
}

bool PhysicsEngineUtils::ConvexDecomposition(const PhysicsMeshData& meshData, const ConvexDecomposeOptions& params, std::vector<PhysicsMeshData>& convexMeshesData)
{
	if (!gConvexMeshDecomposer)
		return false;
	gConvexMeshDecomposer->Decompose(meshData, params, convexMeshesData);
}

void PhysicsEngineUtils::BuildConvexMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices, PhysicsMeshData& meshdata)
{
	PhysicsConvexUtils::BuildConvexMesh(vertices, indices, meshdata);
}