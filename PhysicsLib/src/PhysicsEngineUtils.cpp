#include "Physics/PhysicsCommon.h"
#include "PhysicsEngine.h"
#include "PhysicsArticulation.h"
#include "ConvexMeshDecomposer.h"
#include "Utility/PhysicsConvexUtils.h"
#include "Utility/PhysicsUtils.h"
#include "PhysicsMaterial.h"
#include "ColliderGeometry.h"
static PhysicsEngine* gPhysicsEngine = nullptr;
static ConvexMeshDecomposer* gConvexMeshDecomposer = nullptr;
static bool gDebugDrawingEnabled = false;
static std::function<bool(uint32_t, uint32_t)> gCollisionFilterCallback = nullptr;

IPhysicsEngine* PhysicsEngineUtils::CreatePhysicsEngine(const PhysicsEngineOptions& options, const bool createConvexDecomposer)
{
	_ASSERT(!gPhysicsEngine);
	gPhysicsEngine = new PhysicsEngine(options);
	gPhysicsEngine->Initialize();
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

PhysicsPtr<ISoftBody> PhysicsEngineUtils::CreateSoftBody(const SoftBodyCreateOptions& options)
{
	return gPhysicsEngine->CreateSoftBody(options);
}

IPhysicsProfiler* PhysicsEngineUtils::GetProfiler()
{
	return gPhysicsEngine->GetProfiler();
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

bool PhysicsEngineUtils::RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, PhysicsRaycastHit& hit)
{
	PhysicsPtr<IPhysicsScene> activeScene = gPhysicsEngine ? dynamic_cast<PhysicsEngine*>(gPhysicsEngine)->GetActiveScene() : nullptr;
	if (activeScene)
	{
		activeScene->RaycastSingle(ray, maxDistance, hit);
		return hit.m_Object != nullptr;
	}
	return false;
}

bool PhysicsEngineUtils::RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<PhysicsRaycastHit>& hits)
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
								 const MathLib::HVector3& direction, MathLib::HReal maxDistance, PhysicsRaycastHit& hit)
{
	if (!gPhysicsEngine || !geometry)
		return false;

	PhysicsPtr<IPhysicsScene> activeScene = dynamic_cast<PhysicsEngine*>(gPhysicsEngine)->GetActiveScene();
	if (!activeScene)
		return false;

	MathLib::HVector3 center = startTransform.translation();
	MathLib::HRay3D ray(center, direction.normalized());
	
	activeScene->RaycastSingle(ray, maxDistance, hit);
	return hit.m_Object != nullptr;
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
		gPhysicsEngine->SetDebugRenderer(renderer);
}

PhysicsPtr<IPhysicsDebugRenderer> PhysicsEngineUtils::GetDebugRenderer()
{
	if (gPhysicsEngine)
		return gPhysicsEngine->GetDebugRenderer();
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

// Articulation utility functions
PhysicsPtr<IArticulation> PhysicsEngineUtils::CreateArticulation(const ArticulationCreateOptions &options)
{
	if (!gPhysicsEngine)
		return nullptr;
	return gPhysicsEngine->CreateArticulation(options);
}

PhysicsPtr<IArticulationLink> PhysicsEngineUtils::CreateArticulationLink(PhysicsPtr<IArticulation> articulation, PhysicsPtr<IArticulationLink> parent, const ArticulationLinkCreateOptions &options)
{
	if (!articulation)
		return nullptr;
	return articulation->CreateLink(parent, options);
}

PhysicsPtr<IArticulationJoint> PhysicsEngineUtils::CreateArticulationJoint(PhysicsPtr<IArticulationLink> link, const ArticulationJointCreateOptions &options)
{
	if (!link)
		return nullptr;
	
	// Create joint based on the link's inbound joint
	IArticulationJoint* joint = link->GetInboundJoint();
	if (joint)
	{
		// Configure the existing joint
		joint->SetParentPose(options.parentPose);
		joint->SetChildPose(options.childPose);
		joint->SetFrictionCoefficient(options.frictionCoefficient);
		joint->SetMaxJointVelocity(options.maxJointVelocity);
	}
	
	return std::make_shared<PhysicsArticulationJoint>(link, options);
}

PhysicsPtr<IArticulationCache> PhysicsEngineUtils::CreateArticulationCache(PhysicsPtr<IArticulation> articulation)
{
	if (!articulation)
		return nullptr;
	return articulation->CreateCache();
}

PhysicsPtr<IRigidStatic> PhysicsEngineUtils::CreateRigidStatic(const PhysicsObjectCreateOptions &options)
{
	PhysicsObjectCreateOptions staticOptions = options;
	staticOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
	auto object = CreateObject(staticOptions);
	return std::dynamic_pointer_cast<IRigidStatic>(object);
}

PhysicsPtr<IRigidDynamic> PhysicsEngineUtils::CreateRigidDynamic(const PhysicsObjectCreateOptions &options)
{
	PhysicsObjectCreateOptions dynamicOptions = options;
	dynamicOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
	auto object = CreateObject(dynamicOptions);
	return std::dynamic_pointer_cast<IRigidDynamic>(object);
}

void PhysicsEngineUtils::SetDebugRender(PhysicsPtr<IPhysicsDebugRender> render)
{
	if (gPhysicsEngine)
		gPhysicsEngine->SetDebugRender(render);
}

PhysicsPtr<IPhysicsDebugRender> PhysicsEngineUtils::GetDebugRender()
{
	if (gPhysicsEngine)
		return gPhysicsEngine->GetDebugRender();
	return nullptr;
}