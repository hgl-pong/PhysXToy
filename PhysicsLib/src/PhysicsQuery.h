#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
    class PxScene;
    class PxQueryFilterCallback;
    class PxQueryFilterData;
    class PxRaycastHit;
}

class ISceneQueryFilterCallback
{
public:
    virtual ~ISceneQueryFilterCallback() = default;
    virtual bool PreFilter(const PhysicsPtr<IPhysicsObject>& object) = 0;
    virtual bool PostFilter(const PhysicsPtr<IPhysicsObject>& object, const PhysicsRaycastHit& hit) = 0;
};

struct SceneQueryOptions
{
    uint32_t filterMask = static_cast<uint32_t>(QueryFilterFlag::ALL);
    bool hitBackFaces = false;
    bool hitTriggers = false;
    MathLib::HReal maxDistance = FLT_MAX;
    ISceneQueryFilterCallback* filterCallback = nullptr;
};

struct OverlapSphereOptions : public SceneQueryOptions
{
    MathLib::HVector3 center;
    MathLib::HReal radius;
};

struct OverlapBoxOptions : public SceneQueryOptions
{
    MathLib::HVector3 center;
    MathLib::HVector3 halfExtents;
    MathLib::HQuaternion rotation;
};

struct OverlapCapsuleOptions : public SceneQueryOptions
{
    MathLib::HVector3 point0;
    MathLib::HVector3 point1;
    MathLib::HReal radius;
};

struct RaycastOptions : public SceneQueryOptions
{
    MathLib::HRay3D ray;
};

struct BatchRaycastData
{
    MathLib::HRay3D ray;
    PhysicsRaycastHit hit;
    bool hasHit = false;
};

struct SweepOptions : public SceneQueryOptions
{
    PhysicsPtr<IColliderGeometry> geometry;
    MathLib::HTransform3 startTransform;
    MathLib::HVector3 direction;
};

struct BatchSweepData
{
    PhysicsPtr<IColliderGeometry> geometry;
    MathLib::HTransform3 startTransform;
    MathLib::HVector3 direction;
    PhysicsRaycastHit hit;
    bool hasHit = false;
};

class SceneQuery
{
public:
    static bool RaycastSingle(const RaycastOptions& options, PhysicsRaycastHit& hit);
    static bool RaycastAll(const RaycastOptions& options, std::vector<PhysicsRaycastHit>& hits);
    static bool SweepSingle(const SweepOptions& options, PhysicsRaycastHit& hit);
    static bool SweepAll(const SweepOptions& options, std::vector<PhysicsRaycastHit>& hits);
    static bool OverlapSphere(const OverlapSphereOptions& options, std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects);
    static bool OverlapBox(const OverlapBoxOptions& options, std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects);
    static bool OverlapCapsule(const OverlapCapsuleOptions& options, std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects);
    
    static void BatchRaycast(std::vector<BatchRaycastData>& batchData, const SceneQueryOptions& commonOptions = SceneQueryOptions());
    static void BatchSweep(std::vector<BatchSweepData>& batchData, const SceneQueryOptions& commonOptions = SceneQueryOptions());
    
    static PhysicsPtr<IPhysicsObject> FindObjectFromActor(void* userData);
private:
    static physx::PxScene* GetActiveScene();
    static void ConfigureQueryFilterData(physx::PxQueryFilterData& filterData, const SceneQueryOptions& options);
    static physx::PxQueryFilterCallback* CreateFilterCallback(ISceneQueryFilterCallback* callback);
    static void FillHitResult(const physx::PxRaycastHit& pxHit, PhysicsRaycastHit& hit);
};
