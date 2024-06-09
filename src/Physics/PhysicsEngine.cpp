#include "PhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "physx/cooking/PxCooking.h"
#include "PhysicsScene.h"
#include "PhysicsObject.h"
#include "PhysicsMaterial.h"
#include "ColliderGeometry.h"
#include "Utility/PhysxUtils.h"
#include <assert.h>

#ifndef NDEBUG
#define ENABLE_PVD
#endif

#define PHYSX_PVD_HOST "127.0.0.1"
using namespace physx;

PhysicsEngine::PhysicsEngine(const PhysicsEngineOptions &options)
{
	m_AllocatorCallback = nullptr;
	m_ErrorCallback = nullptr;
	m_Foundation = nullptr;
	m_Physics = nullptr;
	m_Pvd = nullptr;
	m_CpuDispatcher = nullptr;
	m_bInitialized = false;

	m_Options = options;
#ifdef ENABLE_PVD
	m_Options.m_bEnablePVD = true;
#else
	m_Options.m_bEnablePVD = false;
#endif

	// Init Physx
	{
		m_AllocatorCallback = std::make_unique<PxDefaultAllocator>();
		m_ErrorCallback = std::make_unique<PxDefaultErrorCallback>();

		m_Foundation = make_physx_ptr(PxCreateFoundation(PX_PHYSICS_VERSION, *m_AllocatorCallback, *m_ErrorCallback));

		if (m_Options.m_bEnablePVD)
		{
			m_Pvd = make_physx_ptr(PxCreatePvd(*m_Foundation));
			PxPvdTransport *transport = PxDefaultPvdSocketTransportCreate(PHYSX_PVD_HOST, 5425, 10);
			m_Pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
		}
		PxTolerancesScale toleranceScale;
		PxCookingParams cookingParams(toleranceScale);

		m_Physics = make_physx_ptr(PxCreatePhysics(PX_PHYSICS_VERSION, *m_Foundation, toleranceScale, true, m_Pvd.get()));
		m_CpuDispatcher = std::unique_ptr<PxCpuDispatcher>(PxDefaultCpuDispatcherCreate(options.m_NumThreads == 0 ? DEFAULT_CPU_DISPATCHER_NUM_THREADS : options.m_NumThreads));
	}

	m_bInitialized = true;
}

PhysicsEngine::~PhysicsEngine()
{
	m_CpuDispatcher.reset();
	m_Physics.reset();
	if (m_Pvd)
	{
		PxPvdTransport *transport = m_Pvd->getTransport();
		m_Pvd->disconnect();
		m_Pvd.reset();
		PX_RELEASE(transport);
	}
	m_Foundation.reset();
	m_bInitialized = false;
}

PhysicsPtr<IPhysicsObject> PhysicsEngine::CreateObject(const PhysicsObjectCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	PhysicsPtr<IPhysicsMaterial> material = CreateMaterial(options.m_MaterialOptions);
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
	return make_physics_ptr(object);
}

PhysicsPtr<IPhysicsMaterial> PhysicsEngine::CreateMaterial(const PhysicsMaterialCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	return make_physics_ptr(new PhysicsMaterial(options));
}

PhysicsPtr<IPhysicsScene> PhysicsEngine::CreateScene(const PhysicsSceneCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	PhysicsPtr<IPhysicsScene> scene = make_physics_ptr(new PhysicsScene(options, m_CpuDispatcher.get()));
	return scene;
}

PhysicsPtr<IColliderGeometry> PhysicsEngine::CreateColliderGeometry(const CollisionGeometryCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	IColliderGeometry *geometry = nullptr;
	switch (options.m_GeometryType)
	{
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
	{
		geometry = new BoxColliderGeometry(options.m_BoxParams.m_HalfExtents);
		geometry->SetScale(options.m_Scale);
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
	{
		geometry = new SphereColliderGeometry(options.m_SphereParams.m_Radius);
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
	{
		geometry = new PlaneColliderGeometry(options.m_PlaneParams.m_Normal, options.m_PlaneParams.m_Distance);
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
	{
		geometry = new CapsuleColliderGeometry(options.m_CapsuleParams.m_Radius, options.m_CapsuleParams.m_HalfHeight);
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
	{
		geometry = new TriangleMeshColliderGeometry(options.m_TriangleMeshParams.m_Vertices, options.m_TriangleMeshParams.m_Indices);
		break;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
	{
		geometry = new ConvexMeshColliderGeometry(options.m_ConvexMeshParams.m_Vertices, options.m_ConvexMeshParams.m_Indices);
		break;
	}
	default:
		break;
	}
	geometry->SetScale(options.m_Scale);
	return make_physics_ptr(geometry);
}

void PhysicsEngine::SetSolverIterationCount(uint32_t count)
{
	if (!m_bInitialized)
		return;
	m_Options.m_SolverIterationCount = count;
}

uint32_t PhysicsEngine::GetSolverIterationCount() const
{
	if (!m_bInitialized)
		return 0;
	return m_Options.m_SolverIterationCount;
}
