#pragma once
#include "Physics/PhysicsCommon.h"
#include "physx/extensions/PxDefaultCpuDispatcher.h"
namespace physx
{
	class PxAllocatorCallback;
	class PxErrorCallback;
	class PxPvd;
	class PxFoundation;
	class PxPhysics;
	class PxCpuDispatcher;
	class PxCooking;
	class PxGeometry;
	class PxMaterial;
};

class PhysicsAllocator;
class PhysicsErrorCallback;

class PhysicsEngine : public IPhysicsEngine
{
private:
	PhysicsEngine(const PhysicsEngineOptions &options);
	~PhysicsEngine();
public:
	IPhysicsObject *CreateObject(const PhysicsObjectCreateOptions &options) override;
	IPhysicsMaterial *CreateMaterial(const PhysicsMaterialCreateOptions &options) override;
	IPhysicsScene *CreateScene(const PhysicsSceneCreateOptions &options) override;
	IColliderGeometry *CreateColliderGeometry(const CollisionGeometryCreateOptions &options) override;

private:
	friend class PhysicsEngineUtils;
	PhysicsEngineOptions m_Options;
	physx::PxAllocatorCallback* m_AllocatorCallback;
	physx::PxErrorCallback* m_ErrorCallback;
	physx::PxCooking* m_Cooking;
	physx::PxPvd* m_Pvd;
	physx::PxFoundation* m_Foundation;
	physx::PxPhysics* m_Physics;
	physx::PxCpuDispatcher* m_CpuDispatcher;
	bool m_bInitialized;

};


