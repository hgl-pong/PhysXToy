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
	PhysicsPtr<IPhysicsObject> CreateObject(const PhysicsObjectCreateOptions &options) override;
	PhysicsPtr<IPhysicsMaterial> CreateMaterial(const PhysicsMaterialCreateOptions &options) override;
	PhysicsPtr<IPhysicsScene> CreateScene(const PhysicsSceneCreateOptions &options) override;
	PhysicsPtr<IColliderGeometry> CreateColliderGeometry(const CollisionGeometryCreateOptions &options) override;
	void SetSolverIterationCount(uint32_t count) override;
	uint32_t GetSolverIterationCount() const override;

private:
	friend class PhysicsEngineUtils;
	PhysicsEngineOptions m_Options;
	std::unique_ptr<physx::PxAllocatorCallback> m_AllocatorCallback;
	std::unique_ptr<physx::PxErrorCallback> m_ErrorCallback;
	PhysXPtr<physx::PxPvd> m_Pvd;
	PhysXPtr<physx::PxFoundation> m_Foundation;
	PhysXPtr<physx::PxPhysics> m_Physics;
	std::unique_ptr<physx::PxCpuDispatcher> m_CpuDispatcher;

	bool m_bInitialized;

	// resource
	std::vector<PhysicsPtr<IPhysicsScene>> m_Scenes;
};
