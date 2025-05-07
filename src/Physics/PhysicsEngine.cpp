#include "PhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "physx/cooking/PxCooking.h"
#include "PhysicsScene.h"
#include "PhysicsRigid.h"
#include "PhysicsMaterial.h"
#include "ColliderGeometry.h"
#include "Utility/PhysxUtils.h"
#include "PhysicsJoint.h"
#include "PhysicsSoftBody.h"
#include "PhysicsCloth.h"
#include "PhysicsProfiler.h"

#include <assert.h>

#ifndef NDEBUG
#define ENABLE_PVD
#endif

#define PHYSX_PRINT_WARNING(msg) PxGetFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, msg)

using namespace physx;

PhysicsEngine::PhysicsEngine(const PhysicsEngineOptions &options)
{
	m_AllocatorCallback = nullptr;
	m_ErrorCallback = nullptr;
	m_Foundation = nullptr;
	m_Physics = nullptr;
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

		m_Profiler = std::make_unique<PhysicsProfiler>(m_Options.m_bEnablePVD);
		PxTolerancesScale toleranceScale;
		m_Physics = make_physx_ptr(PxCreatePhysics(PX_PHYSICS_VERSION, *m_Foundation, toleranceScale, false, m_Profiler->GetPVD()));
		m_CpuDispatcher = std::unique_ptr<physx::PxCpuDispatcher>(PxDefaultCpuDispatcherCreate(m_Options.m_NumThreads));
	
		physx::PxCudaContextManagerDesc cudaContextManagerDesc;
		m_CudaContextManager = make_physx_ptr(PxCreateCudaContextManager(*m_Foundation, cudaContextManagerDesc, PxGetProfilerCallback()));
		
		if (!m_CudaContextManager || !m_CudaContextManager->contextIsValid())
		{
			PHYSX_PRINT_WARNING("CUDA initialization failed. SoftBody features will be unavailable.");
		}
	}
	
	m_bInitialized = true;
}

PhysicsEngine::~PhysicsEngine()
{
	m_CpuDispatcher.reset();
	m_Physics.reset();
	m_Profiler.reset();
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
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD:
	{
		geometry = new HeightFieldColliderGeometry(
			options.m_HeightFieldParams.m_HeightData,
			options.m_HeightFieldParams.m_Rows,
			options.m_HeightFieldParams.m_Columns,
			options.m_HeightFieldParams.m_RowScale,
			options.m_HeightFieldParams.m_ColumnScale,
			options.m_HeightFieldParams.m_HeightScale
		);
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

PhysicsPtr<IPhysicsJoint> PhysicsEngine::CreateJoint(const JointCreateOptions &options)
{
	if (!m_bInitialized || !m_Physics)
		return nullptr;
	
	if (!options.objectA || !options.objectB)
		return nullptr;

	PhysicsPtr<IPhysicsJoint> joint = std::make_shared<PhysicsJoint>(
		options.type,
		options.objectA,
		options.objectB,
		options.localFrameA,
		options.localFrameB,
		options.collisionEnabled
	);

	return joint;
}

void PhysicsEngine::SetDebugRenderer(PhysicsPtr<IPhysicsDebugRenderer> renderer)
{
	m_DebugRenderer = renderer;
}

PhysicsPtr<IPhysicsDebugRenderer> PhysicsEngine::GetDebugRenderer() const
{
	return m_DebugRenderer;
}

void PhysicsEngine::RegisterCollisionCallback(ICollisionCallback* callback)
{
	if (callback)
		m_CollisionCallbacks.insert(callback);
}

void PhysicsEngine::UnregisterCollisionCallback(ICollisionCallback* callback)
{
	if (callback)
		m_CollisionCallbacks.erase(callback);
}

PhysicsPtr<IPhysicsScene> PhysicsEngine::GetActiveScene() const
{
	return m_ActiveScene;
}

void PhysicsEngine::SetActiveScene(PhysicsPtr<IPhysicsScene> scene)
{
	m_ActiveScene = scene;
}

PhysicsPtr<ISoftBody> PhysicsEngine::CreateSoftBody(const SoftBodyCreateOptions &options)
{
	if (!m_bInitialized)
	{
		return nullptr;
	}
	
	PhysicsPtr<IPhysicsMaterial> material = CreateMaterial(options.m_MaterialOptions);
	if (!material)
	{
		return nullptr;
	}
	
	PhysicsSoftBody* softBody = new PhysicsSoftBody(material);
	
	if (!softBody->CreateFromMesh(options))
	{
		delete softBody;
		return nullptr;
	}
	
	return make_physics_ptr(softBody);
}

PhysicsPtr<ICloth> PhysicsEngine::CreateCloth(const ClothCreateOptions &options)
{
	if (!m_bInitialized)
	{
		return nullptr;
	}
	
	PhysicsPtr<IPhysicsMaterial> material = CreateMaterial(options.m_MaterialOptions);
	if (!material)
	{
		return nullptr;
	}
	
	PhysicsCloth* cloth = new PhysicsCloth(material);
	
	if (!cloth->CreateFromMesh(options))
	{
		delete cloth;
		return nullptr;
	}
	
	return make_physics_ptr(cloth);
}

IPhysicsProfiler* PhysicsEngine::GetProfiler()
{
	return m_Profiler.get();
}

