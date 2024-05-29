#include "PhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "Physics/PhysicsScene.h"
#include "Physics/PhysicsObject.h"
#include "Physics/PhysicsMaterial.h"
#include "Physics/ColliderGeometry.h"
#include "PhysxUtils.h"
#include <assert.h>
#include <minwinbase.h>

#ifndef NDEBUG
#define ENABLE_PVD
#endif

#define PHYSX_PVD_HOST "127.0.0.1"
using namespace physx;

void *_GetFilterShader(const PhysicsSceneFilterShaderType &type) const
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
	ZeroMemory(&m_Options, sizeof(PhysicsEngineOptions));
	gPhysicsEngine = this;
}

PhysicsEngine::~PhysicsEngine()
{
	_ASSERT(gPhysicsEngine);
	gPhysicsEngine = nullptr;
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
		m_AllocatorCallback = std::make_unique<PxDefaultAllocator>();
		m_ErrorCallback = std::make_unique<PxDefaultErrorCallback>();

		m_Foundation = std::make_unique<PxFoundation>(PxCreateFoundation(PX_PHYSICS_VERSION, *m_AllocatorCallback, *m_ErrorCallback));

		if (m_Options.m_bEnablePVD)
		{
			m_Pvd = std::make_unique<PxPvd>(PxCreatePvd(*m_Foundation));
			PxPvdTransport *transport = std::make_unique<PxPvdTransport>(PxDefaultPvdSocketTransportCreate(PHYSX_PVD_HOST, 5425, 10));
			m_Pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
		}
		PxTolerancesScale toleranceScale;
		PxCookingParams cookingParams(toleranceScale);

		m_Physics = std::make_unique<PxPhysics>(PxCreatePhysics(PX_PHYSICS_VERSION, *m_pFoundation, toleranceScale, true, m_Pvd));
		m_Dispatcher = std::make_unique<PxCpuDispatcher>(PxDefaultCpuDispatcherCreate(options.m_iNumThreads == 0 ? 1 : options.m_iNumThreads));
		m_Cooking = std::make_unique<PxCooking>(PxCreateCooking(PX_PHYSICS_VERSION, *m_Foundation, cookingParams));
	}

	m_bInitialized = true;
}

void PhysicsEngine::UnInit()
{
	if (m_pPvd)
	{
		PxPvdTransport *transport = m_Pvd->getTransport();
		m_Pvd->release();
		transport->release();
	}

	m_Dispatcher->release();
	m_Physics->release();
	m_Foundation->release();
	m_Cooking->release();

	m_Pvd.reset();
	m_Dispatcher.reset();
	m_Cooking.reset();
	m_Physics.reset();
	m_Foundation.reset();
	m_ErrorCallback.reset();
	m_AllocatorCallback.reset();
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
	case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
	{
		object = new PhysicsRigidDynamic(material);
		object->SetTransform(options.m_Transform);
		object->break;
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
	material->m_Material = std::make_unique<PxMaterial>(m_Physics->createMaterial(options.m_fStaticFriction, options.m_fDynamicFriction, options.m_fRestitution));
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
	sceneDesc.filterShader = _GetFilterShader(options.m_FilterShaderType);
	scene->m_Scene = std::make_unique<PxScene>(m_Physics->createScene(sceneDesc));
	scene->Init();
	return scene;
}

IColliderGeometry *PhysicsEngine::CreateColliderGeometry(const CollisionGeometryCreateOptions &options)
{
	if (!m_bInitialized)
		return nullptr;
	switch (options.m_Type)
	{
	case CollierGeometryType::Box:
	{
		BoxColliderGeometry *box = new BoxColliderGeometry(options.m_HalfExtents);
		box->SetScale(options.m_Scale);
		return box;
		break;
	}
	case CollierGeometryType::Sphere:
	{
		SphereColliderGeometry *sphere = new SphereColliderGeometry(options.m_Radius);
		sphere->SetScale(options.m_Scale);
		return sphere;
		break;
	}
	default:
		break;
	}

	return nullptr;
}
