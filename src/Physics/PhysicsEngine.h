#pragma once
#include "Physics/PhysicsCommon.h"
#include "Utility/PhysxUtils.h"
#include <set>

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
	class PxCudaContextManager;
};

class PhysicsAllocator;
class PhysicsErrorCallback;
class PhysicsProfiler;

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
	PhysicsPtr<IPhysicsJoint> CreateJoint(const JointCreateOptions &options) override;
	PhysicsPtr<ISoftBody> CreateSoftBody(const SoftBodyCreateOptions &options) override;
	PhysicsPtr<ICloth> CreateCloth(const ClothCreateOptions &options) override;
	IPhysicsProfiler* GetProfiler() override;
	void SetSolverIterationCount(uint32_t count) override;
	uint32_t GetSolverIterationCount() const override;
	void SetDebugRenderer(PhysicsPtr<IPhysicsDebugRenderer> renderer) override;
	PhysicsPtr<IPhysicsDebugRenderer> GetDebugRenderer() const override;
	void RegisterCollisionCallback(ICollisionCallback* callback) override;
	void UnregisterCollisionCallback(ICollisionCallback* callback) override;
	PhysicsPtr<IPhysicsScene> GetActiveScene() const override;
	void SetActiveScene(PhysicsPtr<IPhysicsScene> scene) override;

private:
	friend class PhysicsEngineUtils;
	friend class PhysicsSoftBody;
	PhysicsEngineOptions m_Options;
	std::unique_ptr<physx::PxAllocatorCallback> m_AllocatorCallback;
	std::unique_ptr<physx::PxErrorCallback> m_ErrorCallback;
	PhysXPtr<physx::PxFoundation> m_Foundation;
	PhysXPtr<physx::PxPhysics> m_Physics;
	std::unique_ptr<physx::PxCpuDispatcher> m_CpuDispatcher;
	PhysXPtr<physx::PxCudaContextManager> m_CudaContextManager;
	PhysicsPtr<IPhysicsScene> m_ActiveScene;
	PhysicsPtr<IPhysicsDebugRenderer> m_DebugRenderer;
	std::unique_ptr<PhysicsProfiler> m_Profiler;
	std::unordered_set<ICollisionCallback*> m_CollisionCallbacks;
	bool m_bInitialized;
};
