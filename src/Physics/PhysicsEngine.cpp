#include "Physics/PhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "physx/cooking/PxCooking.h"
#include "Physics/PhysicsScene.h"
#include "Physics/PhysicsObject.h"
#include "Physics/PhysicsMaterial.h"
#include "Physics/ColliderGeometry.h"
#include "PhysxUtils.h"
#include <assert.h>

#ifndef NDEBUG
#define ENABLE_PVD
#endif

#define PHYSX_PVD_HOST "127.0.0.1"
using namespace physx;

PhysicsEngine::PhysicsEngine(const PhysicsEngineOptions& options)
{
	m_AllocatorCallback = nullptr;
	m_ErrorCallback = nullptr;
	m_Foundation = nullptr;
	m_Physics = nullptr;
	m_Pvd = nullptr;
	m_CpuDispatcher = nullptr;
	m_bInitialized = false;
	memset(&m_Options, sizeof(PhysicsEngineOptions),0);

#ifdef ENABLE_PVD
	m_Options.m_bEnablePVD = true;
#else
	m_Options.m_bEnablePVD = false;
#endif

	// Init Physx
	{
		m_AllocatorCallback = new PxDefaultAllocator();
		m_ErrorCallback = new PxDefaultErrorCallback();

		m_Foundation = PxCreateFoundation(PX_PHYSICS_VERSION, *m_AllocatorCallback, *m_ErrorCallback);

		if (m_Options.m_bEnablePVD)
		{
			m_Pvd = PxCreatePvd(*m_Foundation);
			PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PHYSX_PVD_HOST, 5425, 10);
			m_Pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
		}
		PxTolerancesScale toleranceScale;
		PxCookingParams cookingParams(toleranceScale);

		m_Physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_Foundation, toleranceScale, true, m_Pvd);
		m_CpuDispatcher = PxDefaultCpuDispatcherCreate(options.m_iNumThreads == 0 ? DEFAULT_CPU_DISPATCHER_NUM_THREADS : options.m_iNumThreads);
		//m_Cooking = PxCreateCooking(PX_PHYSICS_VERSION, *m_Foundation, cookingParams);
	}

	m_bInitialized = true;
}

PhysicsEngine::~PhysicsEngine()
{
	if (m_Pvd)
	{
		PxPvdTransport *transport = m_Pvd->getTransport();
		PX_RELEASE(m_Pvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(m_Physics); 
	PX_RELEASE(m_Foundation);

	m_bInitialized = false;
}

IPhysicsObject *PhysicsEngine::CreateObject(const PhysicsObjectCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	IPhysicsMaterial *material = CreateMaterial(options.m_MaterialOptions);
	IPhysicsObject *object = nullptr;
	switch (options.m_ObjectType)
	{
	case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
	{
		object = new PhysicsRigidStatic(material);
		object->SetTransform(options.m_Transform);
		break;
	}
	case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
	{
		object = new PhysicsRigidDynamic(material);
		object->SetTransform(options.m_Transform);
		break;
	}
	default:
		break;
	}
	return object;
}

IPhysicsMaterial *PhysicsEngine::CreateMaterial(const PhysicsMaterialCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	return new PhysicsMaterial(options);
}

IPhysicsScene *PhysicsEngine::CreateScene(const PhysicsSceneCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;

	return new PhysicsScene(options,m_CpuDispatcher);
}

IColliderGeometry *PhysicsEngine::CreateColliderGeometry(const CollisionGeometryCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	switch (options.m_GeometryType)
	{
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
	{
		BoxColliderGeometry *box = new BoxColliderGeometry(options.m_BoxParams.m_HalfExtents);
		box->SetScale(options.m_Scale);
		return box;
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
	{
		SphereColliderGeometry *sphere = new SphereColliderGeometry(options.m_SphereParams.m_Radius);
		sphere->SetScale(options.m_Scale);
		return sphere;
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
	{
		PlaneColliderGeometry *plane = new PlaneColliderGeometry(options.m_PlaneParams.m_Normal,options.m_PlaneParams.m_Distance);
		plane->SetScale(options.m_Scale);
		return plane;
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
	{
		CapsuleColliderGeometry *capsule = new CapsuleColliderGeometry(options.m_CapsuleParams.m_Radius,options.m_CapsuleParams.m_HalfHeight);
		capsule->SetScale(options.m_Scale);
		return capsule;
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
	{
		TriangleMeshColliderGeometry *mesh = new TriangleMeshColliderGeometry(options.m_TriangleMeshParams.m_Vertices,options.m_TriangleMeshParams.m_Indices);
		mesh->SetScale(options.m_Scale);
		return mesh;
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
	{
		ConvexMeshColliderGeometry *mesh = new ConvexMeshColliderGeometry(options.m_ConvexMeshParams.m_Vertices);
		mesh->SetScale(options.m_Scale);
		return mesh;
		break;
	}
	default:
		break;
	}

	return nullptr;
}
