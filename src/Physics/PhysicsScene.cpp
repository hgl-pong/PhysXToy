#include "PhysicsScene.h"
#include "PxPhysicsAPI.h"
#include "PhysicsRigid.h"
#include "Utility/PhysXUtils.h"
#include "Utility/PhysicsUtils.h"
#include "PhysicsEngine.h"
#include "PhysicsJoint.h"
#include "PhysicsSoftBody.h"
#include "common/PxRenderBuffer.h"
#include "PhysicsProfiler.h"
#include "PhysicsProfilerScope.h"
#ifndef NDEBUG
#define ENABLE_PVD
#endif
#define PHYSX_PVD_HOST "127.0.0.1"
using namespace physx;

PhysicsScene::PhysicsScene(const PhysicsSceneCreateOptions &options, physx::PxCpuDispatcher *cpuDispatch)
{
    auto &physics = PxGetPhysics();
    PxSceneDesc sceneDesc(physics.getTolerancesScale());
    sceneDesc.gravity = PxVec3(options.m_Gravity[0], options.m_Gravity[1], options.m_Gravity[2]);
    m_Gravity = options.m_Gravity;
    sceneDesc.cpuDispatcher = cpuDispatch;
    sceneDesc.filterShader = GetFilterShader(options.m_FilterShaderType);
    m_Scene = make_physx_ptr<PxScene>(physics.createScene(sceneDesc));
#ifdef ENABLE_PVD
    _ASSERT(m_Scene.get());
    PxPvdSceneClient *pvdClient = m_Scene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }
#endif
}

void PhysicsScene::Release()
{
    m_Scene.reset();
}

void PhysicsScene::Tick(MathLib::HReal deltaTime)
{
    PHYSICS_PROFILE_FRAME("Scene", this);
    
    {
        PHYSICS_PROFILE_ZONE("PhysicsStep", this);
        
        {
            PHYSICS_PROFILE_ZONE("CollisionDetection", this);
            m_Scene->simulate(deltaTime);
        }

        {
            PHYSICS_PROFILE_ZONE("Solver", this);
            m_Scene->fetchResults(true);
        }
        
        PHYSICS_PROFILE_ZONE("Integrate", this);
        
        for (auto &dynamicObject : m_PhysicsRigidDynamics)
        {
            dynamicObject->Update();
        }

        for (auto &softBody : m_PhysicsSoftBodies)
        {
            softBody->Update();
        }
        
        for (auto &cloth : m_PhysicsClothes)
        {
            cloth->Update();
        }
    }
    
    PHYSICS_PROFILE_VALUE("ActiveObjects", (int32_t)m_PhysicsObjects.size(), this);
    PHYSICS_PROFILE_VALUE("ActiveDynamicObjects", (int32_t)m_PhysicsRigidDynamics.size(), this);
    PHYSICS_PROFILE_VALUE("ActiveStaticObjects", (int32_t)m_PhysicsRigidStatics.size(), this);
    PHYSICS_PROFILE_VALUE("ActiveSoftBodies", (int32_t)m_PhysicsSoftBodies.size(), this);
    PHYSICS_PROFILE_VALUE("ActiveJoints", (int32_t)m_Joints.size(), this);
    
    //const PxRenderBuffer& renderBuffer = m_Scene->getRenderBuffer();
    //PHYSICS_PROFILE_VALUE("ContactPoints", (int32_t)renderBuffer.getNbContactPoints(), this);
    //PHYSICS_PROFILE_VALUE("CollisionPairs", (int32_t)renderBuffer.getNbContactPairs(), this);
    //
    //PxSimulationStatistics stats;
    //m_Scene->getSimulationStatistics(stats);
    //uint64_t memoryUsage = stats.gpuMemory + stats.gpuTempMemory;
    //PHYSICS_PROFILE_VALUE("MemoryUsage", (float)memoryUsage, this);
    
    static float accumulatedTime = 0.0f;
    accumulatedTime += deltaTime;
    if (accumulatedTime >= 1.0f)
    {
        PhysicsCacheUtils::CleanupUnusedGeometries();
        accumulatedTime = 0.0f;
    }
    
    PHYSICS_PROFILE_END_FRAME();
}

bool PhysicsScene::AddPhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject)
{
    if (!physicsObject || !physicsObject->IsValid())
    {
        return false;
    }

    m_PhysicsObjects.insert(physicsObject);

    PxActor* actor = nullptr;
    switch (physicsObject->GetType())
    {
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
    {
        auto rigidStatic = static_cast<PhysicsRigidStatic*>(physicsObject.get());
        actor = static_cast<PxActor*>(rigidStatic->GetNativeActor());
        m_PhysicsRigidStatics.push_back(physicsObject);
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
    {
        auto rigidDynamic = static_cast<PhysicsRigidDynamic*>(physicsObject.get());
        actor = static_cast<PxActor*>(rigidDynamic->GetNativeActor());
        m_PhysicsRigidDynamics.push_back(physicsObject);
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_SOFT_BODY:
    {
        auto softBody = std::static_pointer_cast<ISoftBody>(physicsObject);
        actor = reinterpret_cast<PxActor*>(softBody->GetOffset());
        m_PhysicsSoftBodies.push_back(softBody);
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_CLOTH:
    {
        auto cloth = std::static_pointer_cast<ICloth>(physicsObject);
        actor = reinterpret_cast<PxActor*>(cloth->GetOffset());
        m_PhysicsClothes.push_back(physicsObject);
        break;
    }
    default:
        return false;
    }

    if (actor)
    {
        m_Scene->addActor(*actor);
        return true;
    }

    return false;
}

void PhysicsScene::RemovePhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject)
{
    if (!physicsObject || !physicsObject->IsValid())
    {
        return;
    }

    if (m_PhysicsObjects.find(physicsObject) == m_PhysicsObjects.end())
    {
        return;
    }

    PxActor* actor = nullptr;
    switch (physicsObject->GetType())
    {
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
    {
        auto rigidStatic = static_cast<PhysicsRigidStatic*>(physicsObject.get());
        actor = static_cast<PxActor*>(rigidStatic->GetNativeActor());
        auto it = std::find(m_PhysicsRigidStatics.begin(), m_PhysicsRigidStatics.end(), physicsObject);
        if (it != m_PhysicsRigidStatics.end())
        {
            m_PhysicsRigidStatics.erase(it);
        }
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
    {
        auto rigidDynamic = static_cast<PhysicsRigidDynamic*>(physicsObject.get());
        actor = static_cast<PxActor*>(rigidDynamic->GetNativeActor());
        auto it = std::find(m_PhysicsRigidDynamics.begin(), m_PhysicsRigidDynamics.end(), physicsObject);
        if (it != m_PhysicsRigidDynamics.end())
        {
            m_PhysicsRigidDynamics.erase(it);
        }
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_SOFT_BODY:
    {
        auto softBody = std::static_pointer_cast<ISoftBody>(physicsObject);
        actor = reinterpret_cast<PxActor*>(softBody->GetOffset());
        auto it = std::find(m_PhysicsSoftBodies.begin(), m_PhysicsSoftBodies.end(), softBody);
        if (it != m_PhysicsSoftBodies.end())
        {
            m_PhysicsSoftBodies.erase(it);
        }
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_CLOTH:
    {
        auto cloth = std::static_pointer_cast<ICloth>(physicsObject);
        actor = reinterpret_cast<PxActor*>(cloth->GetOffset());
        auto it = std::find(m_PhysicsClothes.begin(), m_PhysicsClothes.end(), physicsObject);
        if (it != m_PhysicsClothes.end())
        {
            m_PhysicsClothes.erase(it);
        }
        break;
    }
    default:
        break;
    }

    if (actor)
    {
        m_Scene->removeActor(*actor);
    }

    m_PhysicsObjects.erase(physicsObject);
}

bool PhysicsScene::AddJoint(PhysicsPtr<IPhysicsJoint> &joint)
{
    if (joint)
    {
        m_Joints.push_back(joint);
        return true;
    }
    return false;
}

void PhysicsScene::RemoveJoint(PhysicsPtr<IPhysicsJoint> &joint)
{
    if (joint)
    {
        auto it = std::find(m_Joints.begin(), m_Joints.end(), joint);
        if (it != m_Joints.end()) {
            m_Joints.erase(it);
        }
    }
}

bool PhysicsScene::AddSoftBody(PhysicsPtr<ISoftBody>& softBody)
{
    if (!m_Scene || !softBody || !softBody->IsValid())
    {
        return false;
    }
    
    PhysicsSoftBody* sb = static_cast<PhysicsSoftBody*>(softBody.get());
    physx::PxSoftBody* pxSoftBody = static_cast<physx::PxSoftBody*>(sb->GetSoftBodyData());
    
    if (!pxSoftBody)
    {
        return false;
    }
    
    m_Scene->addActor(*pxSoftBody);
    m_PhysicsSoftBodies.push_back(softBody);
    m_PhysicsObjects.insert(softBody);
    
    return true;
}

void PhysicsScene::RemoveSoftBody(PhysicsPtr<ISoftBody>& softBody)
{
    if (!m_Scene || !softBody || !softBody->IsValid())
    {
        return;
    }
    
    PhysicsSoftBody* sb = static_cast<PhysicsSoftBody*>(softBody.get());
    physx::PxSoftBody* pxSoftBody = static_cast<physx::PxSoftBody*>(sb->GetSoftBodyData());
    
    if (pxSoftBody)
    {
        m_Scene->removeActor(*pxSoftBody);
    }
    
    auto it = std::find(m_PhysicsSoftBodies.begin(), m_PhysicsSoftBodies.end(), softBody);
    if (it != m_PhysicsSoftBodies.end())
    {
        m_PhysicsSoftBodies.erase(it);
    }
    
    m_PhysicsObjects.erase(softBody);
}

uint32_t PhysicsScene::GetPhysicsObjectCount() const
{
    return m_PhysicsObjects.size();
}

uint32_t PhysicsScene::GetPhysicsRigidDynamicCount() const
{
    return m_PhysicsRigidDynamics.size();
}

uint32_t PhysicsScene::GetPhysicsRigidStaticCount() const
{
    return m_PhysicsRigidStatics.size();
}

uint32_t PhysicsScene::GetJointCount() const
{
    return m_Joints.size();
}

void PhysicsScene::RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, RaycastHit& hit)
{
    hit.object = nullptr;
    hit.collider = nullptr;
    hit.distance = maxDistance;
    hit.position = MathLib::HVector3(0, 0, 0);
    hit.normal = MathLib::HVector3(0, 1, 0);
    
    PxVec3 origin(ray.GetOrigin()[0], ray.GetOrigin()[1], ray.GetOrigin()[2]);
    PxVec3 direction(ray.GetDirection()[0], ray.GetDirection()[1], ray.GetDirection()[2]);
    
    PxRaycastBuffer raycastHit;
    PxQueryFilterData filterData;
    filterData.flags = PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC;
    
    bool hitAny = m_Scene->raycast(origin, direction, maxDistance, raycastHit, PxHitFlag::eDEFAULT, filterData);
    
    if (hitAny)
    {
        auto& closest = raycastHit.block;
        
        hit.distance = closest.distance;
        hit.position = MathLib::HVector3(closest.position.x, closest.position.y, closest.position.z);
        hit.normal = MathLib::HVector3(closest.normal.x, closest.normal.y, closest.normal.z);
        
        if (closest.actor)
        {
            void* userData = closest.actor->userData;
            
            for (auto& object : m_PhysicsRigidStatics)
            {
                if (object->GetUserData() == userData)
                {
                    hit.object = object;
                    break;
                }
            }
            
            if (!hit.object)
            {
                for (auto& object : m_PhysicsRigidDynamics)
                {
                    if (object->GetUserData() == userData)
                    {
                        hit.object = object;
                        break;
                    }
                }
            }
            
            if (hit.object)
            {
                std::vector<PhysicsPtr<IColliderGeometry>> geometries;
                hit.object->GetColliderGeometries(geometries);
                if (!geometries.empty())
                {
                    hit.collider = geometries[0];
                }
            }
        }
    }
}

void PhysicsScene::RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<RaycastHit>& hits)
{
    hits.clear();
    
    PxVec3 origin(ray.GetOrigin()[0], ray.GetOrigin()[1], ray.GetOrigin()[2]);
    PxVec3 direction(ray.GetDirection()[0], ray.GetDirection()[1], ray.GetDirection()[2]);
    
    PxRaycastBuffer raycastHits;
    PxQueryFilterData filterData;
    filterData.flags = PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC;
    
    bool hitAny = m_Scene->raycast(origin, direction, maxDistance, raycastHits, PxHitFlag::eDEFAULT, filterData);
    
    if (hitAny)
    {
        auto& closest = raycastHits.block;
        
        RaycastHit hit;
        hit.distance = closest.distance;
        hit.position = MathLib::HVector3(closest.position.x, closest.position.y, closest.position.z);
        hit.normal = MathLib::HVector3(closest.normal.x, closest.normal.y, closest.normal.z);
        
        if (closest.actor)
        {
            void* userData = closest.actor->userData;
            
            for (auto& object : m_PhysicsRigidStatics)
            {
                if (object->GetUserData() == userData)
                {
                    hit.object = object;
                    break;
                }
            }
            
            if (!hit.object)
            {
                for (auto& object : m_PhysicsRigidDynamics)
                {
                    if (object->GetUserData() == userData)
                    {
                        hit.object = object;
                        break;
                    }
                }
            }
            
            if (hit.object)
            {
                std::vector<PhysicsPtr<IColliderGeometry>> geometries;
                hit.object->GetColliderGeometries(geometries);
                if (!geometries.empty())
                {
                    hit.collider = geometries[0];
                }
            }
            
            hits.push_back(hit);
        }
    }
}

void PhysicsScene::SetGravity(const MathLib::HVector3& gravity)
{
    m_Gravity = gravity;
    if (m_Scene)
    {
        m_Scene->setGravity(PxVec3(gravity[0], gravity[1], gravity[2]));
    }
}

MathLib::HVector3 PhysicsScene::GetGravity() const
{
    return m_Gravity;
}

void PhysicsScene::DebugDraw()
{
    auto physicsEngine = dynamic_cast<PhysicsEngine*>(PhysicsEngineUtils::GetPhysicsEngine());
    if (!physicsEngine)
        return;
        
    auto debugRenderer = physicsEngine->GetDebugRenderer();
    if (!debugRenderer)
        return;
        
    
    for (auto& staticObj : m_PhysicsRigidStatics)
    {
        if (staticObj->IsValid())
        {
            MathLib::HAABBox3D worldBox = staticObj->GetWorldBoundingBox();
            MathLib::HVector3 center = (worldBox.min() + worldBox.max()) * 0.5f;
            MathLib::HVector3 halfExtents = (worldBox.max() - worldBox.min()) * 0.5f;
            
            debugRenderer->DrawBox(center, halfExtents, MathLib::HVector3(0, 1, 0));
            
            debugRenderer->DrawTransform(staticObj->GetTransform(), 1.0f);
        }
    }
    
    for (auto& dynamicObj : m_PhysicsRigidDynamics)
    {
        if (dynamicObj->IsValid())
        {
            MathLib::HAABBox3D worldBox = dynamicObj->GetWorldBoundingBox();
            MathLib::HVector3 center = (worldBox.min() + worldBox.max()) * 0.5f;
            MathLib::HVector3 halfExtents = (worldBox.max() - worldBox.min()) * 0.5f;
            
            debugRenderer->DrawBox(center, halfExtents, MathLib::HVector3(1, 0, 0));
            
            debugRenderer->DrawTransform(dynamicObj->GetTransform(), 1.0f);
        }
    }
    
    for (auto& joint : m_Joints)
    {
        if (joint)
        {
            PhysicsPtr<IPhysicsObject> objA = joint->GetObjectA();
            PhysicsPtr<IPhysicsObject> objB = joint->GetObjectB();
            
            if (objA && objB)
            {
                MathLib::HVector3 posA = objA->GetTransform().translation();
                MathLib::HVector3 posB = objB->GetTransform().translation();
                
                debugRenderer->DrawLine(posA, posB, MathLib::HVector3(0, 0, 1));
            }
        }
    }
    
    debugRenderer->Flush();
}

size_t PhysicsScene::GetOffset() const
{
    return reinterpret_cast<size_t>(m_Scene.get());
}

void* PhysicsScene::GetNativeScene() const
{
    return m_Scene.get();
}

void PhysicsScene::Clear()
{
    if(m_Scene)
    {
        for (auto &object : m_PhysicsObjects)
        {
            if (object && object->IsValid())
            {
                PxActor* actor = nullptr;
                switch (object->GetType())
                {
                case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
                    actor = static_cast<PxActor*>(static_cast<PhysicsRigidStatic*>(object.get())->GetNativeActor());
                    break;
                case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
                    actor = static_cast<PxActor*>(static_cast<PhysicsRigidDynamic*>(object.get())->GetNativeActor());
                    break;
                case PhysicsObjectType::PHYSICS_OBJECT_TYPE_SOFT_BODY:
                    actor = reinterpret_cast<PxActor*>(std::static_pointer_cast<ISoftBody>(object)->GetOffset());
                    break;
                case PhysicsObjectType::PHYSICS_OBJECT_TYPE_CLOTH:
                    actor = reinterpret_cast<PxActor*>(std::static_pointer_cast<ICloth>(object)->GetOffset());
                    break;
                default:
                    break;
                }

                if (actor && m_Scene)
                {
                    m_Scene->removeActor(*actor);
                }
            }
        }
    }

    m_PhysicsObjects.clear();
    m_PhysicsRigidStatics.clear();
    m_PhysicsRigidDynamics.clear();
    m_PhysicsSoftBodies.clear();
    m_PhysicsClothes.clear();
    m_Joints.clear();
}

