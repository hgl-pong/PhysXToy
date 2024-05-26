#pragma once
#include "Physics/PhysicsCommon.h"

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
};

class PhysicsEngine : public IPhysicsEngine
{
public:
	PhysicsEngine();
	~PhysicsEngine();

	void Init(const PhysicsEngineOptions &options) override;
	void UnInit() override;
	IPhysicsObject *CreateObject() override;
	IPhysicsMaterial* CreateMaterial(const PhysicsMaterialCreateOptions& options) override;
	IPhysicsScene *CreateScene(const PhysicsSceneCreateOptions &options) override;
	IColliderGeometry *CreateColliderGeometry(const CollisionGeometryCreateOptions& options) override;

private:
	friend bool CreatePhysXGeometry(PhysicsEngine* engine, const CollisionGeometryCreateOptions &options, PxGeometry** geometry);
	friend bool CreatePhysXMaterial(PhysicsEngine* engine, const PhysicsMaterialCreateOptions& options, PxMaterial** material);
private:
	PhysicsEngineOptions m_Options;
	std::unique_ptr<physx::PxAllocatorCallback> m_AllocatorCallback;
	std::unique_ptr<physx::PxErrorCallback> m_ErrorCallback;
	std::unique_ptr<physx::PxCooking> m_Cooking;
	std::unique_ptr<physx::PxPvd> m_Pvd;
	std::unique_ptr<physx::PxFoundation> m_Foundation;
	std::unique_ptr<physx::PxPhysics> m_Physics;
	std::unique_ptr<physx::PxCpuDispatcher> m_Dispatcher;
	bool m_bInitialized;
};
