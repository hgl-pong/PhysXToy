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

inline void *_GetFilterShader(const PhysicsSceneFilterShaderType &type) 
{
	switch (type)
	{
	case PhysicsSceneFilterShaderType::eDEFAULT:
	{
		return PxDefaultSimulationFilterShader;
		break;
	}
	default:
		break;
	}
	return nullptr;
}

PhysicsEngine::PhysicsEngine()
{
	_ASSERT(!gPhysicsEngine);
	m_AllocatorCallback = nullptr;
	m_ErrorCallback = nullptr;
	m_Foundation = nullptr;
	m_Physics = nullptr;
	m_Dispatcher = nullptr;
	m_Pvd = nullptr;
	m_bInitialized = false;
	memset(&m_Options, sizeof(PhysicsEngineOptions),0);
}

PhysicsEngine::~PhysicsEngine()
{
}

void PhysicsEngine::Init(const PhysicsEngineOptions &options)
{
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
			PxPvdTransport *transport = PxDefaultPvdSocketTransportCreate(PHYSX_PVD_HOST, 5425, 10);
			m_Pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
		}
		PxTolerancesScale toleranceScale;
		PxCookingParams cookingParams(toleranceScale);

		m_Physics = PxCreatePhysics(PX_PHYSICS_VERSION, *m_Foundation, toleranceScale, true, m_Pvd);
		m_Dispatcher = PxDefaultCpuDispatcherCreate(options.m_iNumThreads == 0 ? 1 : options.m_iNumThreads);
		//m_Cooking = std::make_unique<PxCooking>(PxCreateCooking(PX_PHYSICS_VERSION, *m_Foundation.get(), cookingParams));
	}

	m_bInitialized = true;
}

void PhysicsEngine::UnInit()
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

	PhysicsMaterial *material = new PhysicsMaterial();
	material->m_Material = m_Physics->createMaterial(options.m_StaticFriction, options.m_DynamicFriction, options.m_Restitution);
	material->m_Density = options.m_Density;
	return material;
}

IPhysicsScene *PhysicsEngine::CreateScene(const PhysicsSceneCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;

	PhysicsScene *scene = new PhysicsScene();
	const MathLib::HVector3 &gravity = options.m_Gravity;
	PxSceneDesc sceneDesc(m_Physics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(gravity[0], gravity[1], gravity[2]);
	sceneDesc.cpuDispatcher = m_Dispatcher;
	sceneDesc.filterShader = reinterpret_cast<physx::PxSimulationFilterShader>(_GetFilterShader(options.m_FilterShaderType));
	scene->m_Scene = make_physx_ptr<PxScene>(m_Physics->createScene(sceneDesc));
	scene->Init();
	return scene;
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
	default:
		break;
	}

	return nullptr;
}
