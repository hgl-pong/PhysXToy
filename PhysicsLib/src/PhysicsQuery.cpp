#include "PhysicsQuery.h"
#include "PhysicsEngine.h"
#include "PhysicsScene.h"
#include "Utility/PhysX/QueryCallback.h"
#include "Utility/PhysX/CastFilter.h"
#include "Utility/PhysX/SimulationEventCallback.h"

#include <PxPhysicsAPI.h>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultErrorCallback.h>
#include <extensions/PxDefaultCpuDispatcher.h>
#include <foundation/PxFoundation.h>
#include <extensions/PxShapeExt.h>
#include <extensions/PxRigidActorExt.h>
#include <extensions/PxRigidBodyExt.h>
#include <extensions/PxSimpleFactory.h>
#include <vehicle/PxVehicleSDK.h>
#include <vehicle/PxVehicleWheels.h>

using namespace physx;

class CustomQueryFilterCallback : public PxQueryFilterCallback
{
public:
    ISceneQueryFilterCallback* m_UserCallback;

    CustomQueryFilterCallback(ISceneQueryFilterCallback* userCallback) : m_UserCallback(userCallback) {}

    virtual PxQueryHitType::Enum preFilter(const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags) override
    {
        if (!m_UserCallback)
            return PxQueryHitType::eBLOCK;

        void* userData = actor->userData;
        PhysicsPtr<IPhysicsObject> object = SceneQuery::FindObjectFromActor(userData);
        if (!object)
            return PxQueryHitType::eNONE;

        bool shouldProcess = m_UserCallback->PreFilter(object);
        return shouldProcess ? PxQueryHitType::eBLOCK : PxQueryHitType::eNONE;
    }

    virtual PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit, const PxShape* shape, const PxRigidActor* actor) override
    {
        if (!m_UserCallback)
            return PxQueryHitType::eBLOCK;

        if (!actor)
            return PxQueryHitType::eNONE;

        void* userData = actor->userData;
        PhysicsPtr<IPhysicsObject> object = SceneQuery::FindObjectFromActor(userData);
        if (!object)
            return PxQueryHitType::eNONE;

        PhysicsRaycastHit rayHit;
        bool shouldProcess = m_UserCallback->PostFilter(object, rayHit);
        return shouldProcess ? PxQueryHitType::eBLOCK : PxQueryHitType::eNONE;
    }
};

PxScene* SceneQuery::GetActiveScene()
{
    auto physicsEngine = dynamic_cast<PhysicsEngine*>(PhysicsEngineUtils::GetPhysicsEngine());
    if (!physicsEngine)
        return nullptr;

    auto activeScene = physicsEngine->GetActiveScene();
    if (!activeScene)
        return nullptr;

    return static_cast<PxScene*>(activeScene->GetNativeScene());
}

void SceneQuery::ConfigureQueryFilterData(PxQueryFilterData& filterData, const SceneQueryOptions& options)
{
    filterData.data.word0 = options.filterMask;
    filterData.flags = PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER;

    if (options.hitBackFaces)
        filterData.flags |= PxQueryFlag::eNO_BLOCK;

    if (options.hitTriggers)
        filterData.flags |= PxQueryFlag::eANY_HIT;

    if (options.filterCallback)
        filterData.flags |= PxQueryFlag::ePOSTFILTER;
}

PxQueryFilterCallback* SceneQuery::CreateFilterCallback(ISceneQueryFilterCallback* callback)
{
    if (!callback)
        return nullptr;

    return new CustomQueryFilterCallback(callback);
}

PhysicsPtr<IPhysicsObject> SceneQuery::FindObjectFromActor(void* userData)
{
    if (!userData)
        return nullptr;

    auto physicsEngine = dynamic_cast<PhysicsEngine*>(PhysicsEngineUtils::GetPhysicsEngine());
    if (!physicsEngine)
        return nullptr;

    auto scene = physicsEngine->GetActiveScene();
    if (!scene)
        return nullptr;

    return PhysicsPtr<IPhysicsObject>(static_cast<IPhysicsObject*>(userData), PhysicsDeleter<IPhysicsObject>());
}

void SceneQuery::FillHitResult(const PxRaycastHit& pxHit, PhysicsRaycastHit& hit)
{
    hit.m_Distance = pxHit.distance;
    hit.m_Position = MathLib::HVector3(pxHit.position.x, pxHit.position.y, pxHit.position.z);
    hit.m_Normal = MathLib::HVector3(pxHit.normal.x, pxHit.normal.y, pxHit.normal.z);

    if (pxHit.actor)
    {
        hit.m_Object = FindObjectFromActor(pxHit.actor->userData);
        
        if (hit.m_Object && pxHit.shape)
        {
            std::vector<PhysicsPtr<IColliderGeometry>> geometries;
            hit.m_Object->GetColliderGeometries(geometries);
            
            if (!geometries.empty())
            {
                hit.m_Collider = geometries[0];
            }
        }
    }
}

bool SceneQuery::RaycastSingle(const RaycastOptions& options, PhysicsRaycastHit& hit)
{
    PxScene* scene = GetActiveScene();
    if (!scene)
        return false;

    const MathLib::HRay3D& ray = options.ray;
    PxVec3 origin(ray.GetOrigin()[0], ray.GetOrigin()[1], ray.GetOrigin()[2]);
    PxVec3 direction(ray.GetDirection()[0], ray.GetDirection()[1], ray.GetDirection()[2]);

    PxRaycastBuffer raycastHit;
    PxQueryFilterData filterData;
    ConfigureQueryFilterData(filterData, options);

    PxQueryFilterCallback* filterCallback = CreateFilterCallback(options.filterCallback);
    std::unique_ptr<PxQueryFilterCallback, std::function<void(PxQueryFilterCallback*)>> callbackCleaner(
        filterCallback, [](PxQueryFilterCallback* cb) { delete cb; });

    bool hitAny = scene->raycast(
        origin, 
        direction, 
        options.maxDistance, 
        raycastHit, 
        PxHitFlag::eDEFAULT, 
        filterData, 
        filterCallback
    );

    if (hitAny && raycastHit.hasBlock)
    {
        FillHitResult(raycastHit.block, hit);
        return true;
    }

    return false;
}

bool SceneQuery::RaycastAll(const RaycastOptions& options, std::vector<PhysicsRaycastHit>& hits)
{
    hits.clear();

    PxScene* scene = GetActiveScene();
    if (!scene)
        return false;

    const MathLib::HRay3D& ray = options.ray;
    PxVec3 origin(ray.GetOrigin()[0], ray.GetOrigin()[1], ray.GetOrigin()[2]);
    PxVec3 direction(ray.GetDirection()[0], ray.GetDirection()[1], ray.GetDirection()[2]);

    const uint32_t bufferSize = 256;
    PxRaycastHit hitBuffer[bufferSize];
    PxRaycastBuffer raycastHit(hitBuffer, bufferSize);

    PxQueryFilterData filterData;
    ConfigureQueryFilterData(filterData, options);

    PxQueryFilterCallback* filterCallback = CreateFilterCallback(options.filterCallback);
    std::unique_ptr<PxQueryFilterCallback, std::function<void(PxQueryFilterCallback*)>> callbackCleaner(
        filterCallback, [](PxQueryFilterCallback* cb) { delete cb; });

    bool hitAny = scene->raycast(
        origin, 
        direction, 
        options.maxDistance, 
        raycastHit, 
        PxHitFlag::eDEFAULT, 
        filterData, 
        filterCallback
    );

    if (hitAny)
    {
        if (raycastHit.hasBlock)
        {
            PhysicsRaycastHit hit;
            FillHitResult(raycastHit.block, hit);
            hits.push_back(hit);
        }

        const PxU32 nbTouches = raycastHit.nbTouches;
        for (PxU32 i = 0; i < nbTouches; i++)
        {
            PhysicsRaycastHit hit;
            FillHitResult(raycastHit.touches[i], hit);
            hits.push_back(hit);
        }

        return !hits.empty();
    }

    return false;
}

void SceneQuery::BatchRaycast(std::vector<BatchRaycastData>& batchData, const SceneQueryOptions& commonOptions)
{
    PxScene* scene = GetActiveScene();
    if (!scene)
        return;

    for (auto& data : batchData)
    {
        RaycastOptions options;
        options.ray = data.ray;
        options.filterMask = commonOptions.filterMask;
        options.hitBackFaces = commonOptions.hitBackFaces;
        options.hitTriggers = commonOptions.hitTriggers;
        options.maxDistance = commonOptions.maxDistance;
        options.filterCallback = commonOptions.filterCallback;

        data.hasHit = RaycastSingle(options, data.hit);
    }
}

bool SceneQuery::SweepSingle(const SweepOptions& options, PhysicsRaycastHit& hit)
{
    PxScene* scene = GetActiveScene();
    if (!scene || !options.geometry)
        return false;

    PxSphereGeometry pxGeom(1.0f);

    const MathLib::HTransform3& transform = options.startTransform;
    PxTransform pxTransform = ConvertUtils::ToPx(transform);

    PxVec3 pxDirection(options.direction[0], options.direction[1], options.direction[2]);
    
    PxSweepBuffer sweepHit;
    PxQueryFilterData filterData;
    ConfigureQueryFilterData(filterData, options);

    PxQueryFilterCallback* filterCallback = CreateFilterCallback(options.filterCallback);
    std::unique_ptr<PxQueryFilterCallback, std::function<void(PxQueryFilterCallback*)>> callbackCleaner(
        filterCallback, [](PxQueryFilterCallback* cb) { delete cb; });

    bool hitAny = scene->sweep(pxGeom, pxTransform, pxDirection, options.maxDistance,
                              sweepHit, PxHitFlag::eDEFAULT, filterData, filterCallback);

    if (hitAny && sweepHit.hasBlock)
    {
        const PxSweepHit& closest = sweepHit.block;
        
        hit.m_Distance = closest.distance;
        hit.m_Position = MathLib::HVector3(closest.position.x, closest.position.y, closest.position.z);
        hit.m_Normal = MathLib::HVector3(closest.normal.x, closest.normal.y, closest.normal.z);
        
        if (closest.actor)
        {
            hit.m_Object = FindObjectFromActor(closest.actor->userData);
            
            if (hit.m_Object && closest.shape)
            {
                std::vector<PhysicsPtr<IColliderGeometry>> geometries;
                hit.m_Object->GetColliderGeometries(geometries);
                
                if (!geometries.empty())
                {
                    hit.m_Collider = geometries[0];
                }
            }
        }
        
        return true;
    }

    return false;
}

bool SceneQuery::SweepAll(const SweepOptions& options, std::vector<PhysicsRaycastHit>& hits)
{
    hits.clear();

    PxScene* scene = GetActiveScene();
    if (!scene || !options.geometry)
        return false;

    PxSphereGeometry pxGeom(1.0f);

    const MathLib::HTransform3& transform = options.startTransform;
    PxTransform pxTransform = ConvertUtils::ToPx(transform);

    PxVec3 pxDirection(options.direction[0], options.direction[1], options.direction[2]);
    
    const uint32_t bufferSize = 256;
    PxSweepHit hitBuffer[bufferSize];
    PxSweepBuffer sweepHits(hitBuffer, bufferSize);
    
    PxQueryFilterData filterData;
    ConfigureQueryFilterData(filterData, options);

    PxQueryFilterCallback* filterCallback = CreateFilterCallback(options.filterCallback);
    std::unique_ptr<PxQueryFilterCallback, std::function<void(PxQueryFilterCallback*)>> callbackCleaner(
        filterCallback, [](PxQueryFilterCallback* cb) { delete cb; });

    bool hitAny = scene->sweep(pxGeom, pxTransform, pxDirection, options.maxDistance,
                              sweepHits, PxHitFlag::eDEFAULT, filterData, filterCallback);

    if (hitAny)
    {
        if (sweepHits.hasBlock)
        {
            PhysicsRaycastHit hit;
            const PxSweepHit& closest = sweepHits.block;
            
            hit.m_Distance = closest.distance;
            hit.m_Position = MathLib::HVector3(closest.position.x, closest.position.y, closest.position.z);
            hit.m_Normal = MathLib::HVector3(closest.normal.x, closest.normal.y, closest.normal.z);
            
            if (closest.actor)
            {
                hit.m_Object = FindObjectFromActor(closest.actor->userData);
                
                if (hit.m_Object && closest.shape)
                {
                    std::vector<PhysicsPtr<IColliderGeometry>> geometries;
                    hit.m_Object->GetColliderGeometries(geometries);
                    
                    if (!geometries.empty())
                    {
                        hit.m_Collider = geometries[0];
                    }
                }
            }
            
            hits.push_back(hit);
        }

        const PxU32 nbTouches = sweepHits.nbTouches;
        for (PxU32 i = 0; i < nbTouches; i++)
        {
            PhysicsRaycastHit hit;
            const PxSweepHit& touch = sweepHits.touches[i];
            
            hit.m_Distance = touch.distance;
            hit.m_Position = MathLib::HVector3(touch.position.x, touch.position.y, touch.position.z);
            hit.m_Normal = MathLib::HVector3(touch.normal.x, touch.normal.y, touch.normal.z);
            
            if (touch.actor)
            {
                hit.m_Object = FindObjectFromActor(touch.actor->userData);
                
                if (hit.m_Object && touch.shape)
                {
                    std::vector<PhysicsPtr<IColliderGeometry>> geometries;
                    hit.m_Object->GetColliderGeometries(geometries);
                    
                    if (!geometries.empty())
                    {
                        hit.m_Collider = geometries[0];
                    }
                }
            }
            
            hits.push_back(hit);
        }

        return !hits.empty();
    }

    return false;
}

void SceneQuery::BatchSweep(std::vector<BatchSweepData>& batchData, const SceneQueryOptions& commonOptions)
{
    PxScene* scene = GetActiveScene();
    if (!scene)
        return;

    for (auto& data : batchData)
    {
        SweepOptions options;
        options.geometry = data.geometry;
        options.startTransform = data.startTransform;
        options.direction = data.direction;
        options.filterMask = commonOptions.filterMask;
        options.hitBackFaces = commonOptions.hitBackFaces;
        options.hitTriggers = commonOptions.hitTriggers;
        options.maxDistance = commonOptions.maxDistance;
        options.filterCallback = commonOptions.filterCallback;

        data.hasHit = SweepSingle(options, data.hit);
    }
}

bool SceneQuery::OverlapSphere(const OverlapSphereOptions& options, std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects)
{
    overlappingObjects.clear();

    PxScene* scene = GetActiveScene();
    if (!scene)
        return false;

    PxSphereGeometry sphereGeom(options.radius);
    
    PxTransform sphereTransform(
        PxVec3(options.center[0], options.center[1], options.center[2])
    );
    
    const uint32_t bufferSize = 256;
    PxOverlapHit hitBuffer[bufferSize];
    PxOverlapBuffer overlapBuffer(hitBuffer, bufferSize);
    
    PxQueryFilterData filterData;
    ConfigureQueryFilterData(filterData, options);

    PxQueryFilterCallback* filterCallback = CreateFilterCallback(options.filterCallback);
    std::unique_ptr<PxQueryFilterCallback, std::function<void(PxQueryFilterCallback*)>> callbackCleaner(
        filterCallback, [](PxQueryFilterCallback* cb) { delete cb; });

    bool hasOverlap = scene->overlap(sphereGeom, sphereTransform, overlapBuffer, filterData, filterCallback);
    
    if (hasOverlap)
    {
        const PxU32 nbTouches = overlapBuffer.nbTouches;
        for (PxU32 i = 0; i < nbTouches; i++)
        {
            const PxOverlapHit& hit = overlapBuffer.touches[i];
            if (hit.actor)
            {
                PhysicsPtr<IPhysicsObject> object = FindObjectFromActor(hit.actor->userData);
                if (object)
                {
                    overlappingObjects.push_back(object);
                }
            }
        }
        
        return !overlappingObjects.empty();
    }
    
    return false;
}

bool SceneQuery::OverlapBox(const OverlapBoxOptions& options, std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects)
{
    overlappingObjects.clear();

    PxScene* scene = GetActiveScene();
    if (!scene)
        return false;

    PxBoxGeometry boxGeom(
        PxVec3(options.halfExtents[0], options.halfExtents[1], options.halfExtents[2])
    );
    
    PxTransform boxTransform(
        PxVec3(options.center[0], options.center[1], options.center[2]),
        PxQuat(options.rotation.x(), options.rotation.y(), options.rotation.z(), options.rotation.w())
    );
    
    const uint32_t bufferSize = 256;
    PxOverlapHit hitBuffer[bufferSize];
    PxOverlapBuffer overlapBuffer(hitBuffer, bufferSize);
    
    PxQueryFilterData filterData;
    ConfigureQueryFilterData(filterData, options);

    PxQueryFilterCallback* filterCallback = CreateFilterCallback(options.filterCallback);
    std::unique_ptr<PxQueryFilterCallback, std::function<void(PxQueryFilterCallback*)>> callbackCleaner(
        filterCallback, [](PxQueryFilterCallback* cb) { delete cb; });

    bool hasOverlap = scene->overlap(boxGeom, boxTransform, overlapBuffer, filterData, filterCallback);
    
    if (hasOverlap)
    {
        const PxU32 nbTouches = overlapBuffer.nbTouches;
        for (PxU32 i = 0; i < nbTouches; i++)
        {
            const PxOverlapHit& hit = overlapBuffer.touches[i];
            if (hit.actor)
            {
                PhysicsPtr<IPhysicsObject> object = FindObjectFromActor(hit.actor->userData);
                if (object)
                {
                    overlappingObjects.push_back(object);
                }
            }
        }
        
        return !overlappingObjects.empty();
    }
    
    return false;
}

bool SceneQuery::OverlapCapsule(const OverlapCapsuleOptions& options, std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects)
{
    overlappingObjects.clear();

    PxScene* scene = GetActiveScene();
    if (!scene)
        return false;

    PxVec3 p0(options.point0[0], options.point0[1], options.point0[2]);
    PxVec3 p1(options.point1[0], options.point1[1], options.point1[2]);
    
    PxVec3 center = (p0 + p1) * 0.5f;
    PxVec3 direction = p1 - p0;
    float halfHeight = direction.magnitude() * 0.5f;
    
    if (halfHeight < 1e-4f)
    {
        return OverlapSphere(OverlapSphereOptions{
            options.filterMask,
            options.hitBackFaces,
            options.hitTriggers,
            options.maxDistance,
            options.filterCallback,
            MathLib::HVector3(center.x, center.y, center.z),
            options.radius
        }, overlappingObjects);
    }
    
    direction.normalize();
    
    PxQuat rotation = PxShortestRotation(PxVec3(0, 1, 0), direction);
    
    PxCapsuleGeometry capsuleGeom(options.radius, halfHeight);
    
    PxTransform capsuleTransform(center, rotation);
    
    const uint32_t bufferSize = 256;
    PxOverlapHit hitBuffer[bufferSize];
    PxOverlapBuffer overlapBuffer(hitBuffer, bufferSize);
    
    PxQueryFilterData filterData;
    ConfigureQueryFilterData(filterData, options);

    PxQueryFilterCallback* filterCallback = CreateFilterCallback(options.filterCallback);
    std::unique_ptr<PxQueryFilterCallback, std::function<void(PxQueryFilterCallback*)>> callbackCleaner(
        filterCallback, [](PxQueryFilterCallback* cb) { delete cb; });

    bool hasOverlap = scene->overlap(capsuleGeom, capsuleTransform, overlapBuffer, filterData, filterCallback);
    
    if (hasOverlap)
    {
        const PxU32 nbTouches = overlapBuffer.nbTouches;
        for (PxU32 i = 0; i < nbTouches; i++)
        {
            const PxOverlapHit& hit = overlapBuffer.touches[i];
            if (hit.actor)
            {
                PhysicsPtr<IPhysicsObject> object = FindObjectFromActor(hit.actor->userData);
                if (object)
                {
                    overlappingObjects.push_back(object);
                }
            }
        }
        
        return !overlappingObjects.empty();
    }
    
    return false;
}