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
};

class PhysicsEngine : public IPhysicsEngine
{
public:
	PhysicsEngine();
	~PhysicsEngine();

	void Init(const PhysicsEngineOptions &options) override;
	void UnInit() override;
	IPhysicsObject *CreateObject() override;
	IPhyicsMaterial* CreateMaterial(const PhysicsMaterialCreateOptions& options) override;
	IPhysicsScene *CreateScene(const PhysicsSceneCreateOptions &options) override;
	IColliderGeometry *CreateColliderGeometry() override;

public:

private:
	PhysicsEngineOptions m_Options;

	std::unique_ptr<physx::PxAllocatorCallback> m_AllocatorCallback;
	std::unique_ptr<physx::PxErrorCallback> m_ErrorCallback;
	std::unique_ptr<physx::PxPvd> m_Pvd;
	std::unique_ptr<physx::PxFoundation> m_Foundation;
	std::unique_ptr<physx::PxPhysics> m_Physics;
	std::unique_ptr<physx::PxCpuDispatcher> m_Dispatcher;
	bool m_bInitialized;
};