#include "Physics/PhysicsCommon.h"
#include "PhysicsEngine.h"
#include "ConvexMeshDecomposer.h"
#include "Utility/PhysicsConvexUtils.h"
static PhysicsEngine* gPhysicsEngine = nullptr;
static ConvexMeshDecomposer* gConvexMeshDecomposer = nullptr;
static bool gDebugDrawingEnabled = false;
static std::function<bool(uint32_t, uint32_t)> gCollisionFilterCallback = nullptr;

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

PhysicsPtr<IPhysicsJoint> PhysicsEngineUtils::CreateJoint(const JointCreateOptions& options)
{
	return gPhysicsEngine->CreateJoint(options);
}

bool PhysicsEngineUtils::ConvexDecomposition(const PhysicsMeshData& meshData, const ConvexDecomposeOptions& params, std::vector<PhysicsMeshData>& convexMeshesData)
{
	if (!gConvexMeshDecomposer)
		return false;
	return gConvexMeshDecomposer->Decompose(meshData, params, convexMeshesData);
}

void PhysicsEngineUtils::BuildConvexMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices, PhysicsMeshData& meshdata)
{
	PhysicsConvexUtils::BuildConvexMesh(vertices, indices, meshdata);
}

bool PhysicsEngineUtils::RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, RaycastHit& hit)
{
	PhysicsPtr<IPhysicsScene> activeScene = gPhysicsEngine ? dynamic_cast<PhysicsEngine*>(gPhysicsEngine)->GetActiveScene() : nullptr;
	if (activeScene)
	{
		activeScene->RaycastSingle(ray, maxDistance, hit);
		return hit.object != nullptr;
	}
	return false;
}

bool PhysicsEngineUtils::RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<RaycastHit>& hits)
{
	PhysicsPtr<IPhysicsScene> activeScene = gPhysicsEngine ? dynamic_cast<PhysicsEngine*>(gPhysicsEngine)->GetActiveScene() : nullptr;
	if (activeScene)
	{
		activeScene->RaycastAll(ray, maxDistance, hits);
		return !hits.empty();
	}
	return false;
}

bool PhysicsEngineUtils::SweepTest(PhysicsPtr<IColliderGeometry> geometry, const MathLib::HTransform3& startTransform, 
								 const MathLib::HVector3& direction, MathLib::HReal maxDistance, RaycastHit& hit)
{
	if (!gPhysicsEngine || !geometry)
		return false;

	PhysicsPtr<IPhysicsScene> activeScene = dynamic_cast<PhysicsEngine*>(gPhysicsEngine)->GetActiveScene();
	if (!activeScene)
		return false;

	MathLib::HVector3 center = startTransform.translation();
	MathLib::HRay3D ray(center, direction.normalized());
	
	activeScene->RaycastSingle(ray, maxDistance, hit);
	return hit.object != nullptr;
}

bool PhysicsEngineUtils::BoxOverlap(const MathLib::HVector3& center, const MathLib::HVector3& halfExtents, 
								  std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects)
{
	if (!gPhysicsEngine)
		return false;

	CollisionGeometryCreateOptions options;
	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
	options.m_BoxParams.m_HalfExtents = halfExtents;
	
	PhysicsPtr<IColliderGeometry> boxGeometry = gPhysicsEngine->CreateColliderGeometry(options);
	if (!boxGeometry)
		return false;
		
	MathLib::HAABBox3D aabb(center - halfExtents, center + halfExtents);
	
	PhysicsPtr<IPhysicsScene> activeScene = dynamic_cast<PhysicsEngine*>(gPhysicsEngine)->GetActiveScene();
	if (!activeScene)
		return false;
		
	std::vector<PhysicsPtr<IPhysicsObject>> sceneObjects;
	
	for (auto& object : sceneObjects)
	{
		if (object->GetWorldBoundingBox().intersects(aabb))
		{
			overlappingObjects.push_back(object);
		}
	}
	
	return !overlappingObjects.empty();
}

bool PhysicsEngineUtils::SphereOverlap(const MathLib::HVector3& center, MathLib::HReal radius, 
									 std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects)
{
	if (!gPhysicsEngine)
		return false;

	CollisionGeometryCreateOptions options;
	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
	options.m_SphereParams.m_Radius = radius;
	
	PhysicsPtr<IColliderGeometry> sphereGeometry = gPhysicsEngine->CreateColliderGeometry(options);
	if (!sphereGeometry)
		return false;
		
	MathLib::HVector3 extent(radius, radius, radius);
	MathLib::HAABBox3D aabb(center - extent, center + extent);
	
	PhysicsPtr<IPhysicsScene> activeScene = dynamic_cast<PhysicsEngine*>(gPhysicsEngine)->GetActiveScene();
	if (!activeScene)
		return false;
		
	std::vector<PhysicsPtr<IPhysicsObject>> sceneObjects;
	
	for (auto& object : sceneObjects)
	{
		if (object->GetWorldBoundingBox().intersects(aabb))
		{
			MathLib::HVector3 objectCenter = object->GetTransform().translation();
			MathLib::HReal distanceSq = (objectCenter - center).norm();
			
			if (distanceSq <= radius * radius)
			{
				overlappingObjects.push_back(object);
			}
		}
	}
	
	return !overlappingObjects.empty();
}

void PhysicsEngineUtils::EnableDebugDrawing(bool enable)
{
	gDebugDrawingEnabled = enable;
}

bool PhysicsEngineUtils::IsDebugDrawingEnabled()
{
	return gDebugDrawingEnabled;
}

void PhysicsEngineUtils::SetDebugRenderer(PhysicsPtr<IPhysicsDebugRenderer> renderer)
{
	if (gPhysicsEngine)
	{
		gPhysicsEngine->SetDebugRenderer(renderer);
	}
}

PhysicsPtr<IPhysicsDebugRenderer> PhysicsEngineUtils::GetDebugRenderer()
{
	if (gPhysicsEngine)
	{
		return gPhysicsEngine->GetDebugRenderer();
	}
	return nullptr;
}

void PhysicsEngineUtils::SetCollisionFilterCallback(std::function<bool(uint32_t, uint32_t)> callback)
{
	gCollisionFilterCallback = callback;
}

bool PhysicsEngineUtils::DefaultCollisionFilter(uint32_t layerA, uint32_t layerB)
{
	if (gCollisionFilterCallback)
	{
		return gCollisionFilterCallback(layerA, layerB);
	}
	
	return true;
}