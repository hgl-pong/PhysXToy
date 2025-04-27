#include "PhysicsScene.h"
#include "PxPhysicsAPI.h"
#include "PhysicsRigid.h"
#include "Utility/PhysXUtils.h"
#include "PhysicsEngine.h"
#include "PhysicsJoint.h"
#include "PhysicsSoftBody.h"
#include "physx/common/PxRenderBuffer.h"
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
    m_Scene->simulate(deltaTime);
    m_Scene->fetchResults(true);

    for (auto &dynamicObject : m_PhysicsRigidDynamics)
    {
        dynamicObject->Update();
    }

    for (auto &softBody : m_PhysicsSoftBodies)
    {
        softBody->Update();
    }
}

bool PhysicsScene::AddPhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject)
{
    const size_t offset = physicsObject->GetOffset();
    bool result = false;
    switch (physicsObject->GetType())
    {
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
    {
        PxRigidStatic *pRigidStatic = reinterpret_cast<PhysXPtr<PxRigidStatic>*>(reinterpret_cast<char *>(physicsObject.get()) + offset)->get();
        if (pRigidStatic->getNbShapes() == 0)
            return false;
        if (m_Scene->addActor(*pRigidStatic)) {
            m_PhysicsRigidStatics.push_back(std::static_pointer_cast<IRigidStatic>(physicsObject));
            result = true;
        }
        if (!result)
            m_Scene->removeActor(*pRigidStatic);
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
    {
        PxRigidDynamic *pRigidDynamic = reinterpret_cast<PhysXPtr<PxRigidDynamic> *>(reinterpret_cast<char *>(physicsObject.get()) + offset)->get();
        if (pRigidDynamic->getNbShapes() == 0)
            return false;
        if (m_Scene->addActor(*pRigidDynamic)) {
            m_PhysicsRigidDynamics.push_back(std::static_pointer_cast<IRigidDynamic>(physicsObject));
            result = true;
        }
        if (!result)
            m_Scene->removeActor(*pRigidDynamic);
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_SOFT_BODY:
    {
        PhysicsPtr<ISoftBody> softBody = std::static_pointer_cast<ISoftBody>(physicsObject);
        return AddSoftBody(softBody);
    }
    default:
        break;
    }
    return result;
}

void PhysicsScene::RemovePhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject)
{
    // m_Scene->removeActor(physicsObject->GetPhysicsObject());
    switch (physicsObject->GetType())
    {
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
    {
        auto staticObj = std::static_pointer_cast<IRigidStatic>(physicsObject);
        auto it = std::find(m_PhysicsRigidStatics.begin(), m_PhysicsRigidStatics.end(), staticObj);
        if (it != m_PhysicsRigidStatics.end()) {
            m_PhysicsRigidStatics.erase(it);
        }
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
    {
        auto dynamicObj = std::static_pointer_cast<IRigidDynamic>(physicsObject);
        auto it = std::find(m_PhysicsRigidDynamics.begin(), m_PhysicsRigidDynamics.end(), dynamicObj);
        if (it != m_PhysicsRigidDynamics.end()) {
            m_PhysicsRigidDynamics.erase(it);
        }
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_SOFT_BODY:
    {
        auto softBody = std::static_pointer_cast<ISoftBody>(physicsObject);
        RemoveSoftBody(softBody);
        return;
    }
    default:
        break;
    }
}

bool PhysicsScene::AddJoint(PhysicsPtr<IPhysicsJoint> &joint)
{
    if (joint)
    {
        m_PhysicsJoints.push_back(joint);
        return true;
    }
    return false;
}

void PhysicsScene::RemoveJoint(PhysicsPtr<IPhysicsJoint> &joint)
{
    if (joint)
    {
        auto it = std::find(m_PhysicsJoints.begin(), m_PhysicsJoints.end(), joint);
        if (it != m_PhysicsJoints.end()) {
            m_PhysicsJoints.erase(it);
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
    m_PhysicsObjects.push_back(softBody);
    
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
    
    // 从列表中移除
    auto it = std::find(m_PhysicsSoftBodies.begin(), m_PhysicsSoftBodies.end(), softBody);
    if (it != m_PhysicsSoftBodies.end())
    {
        m_PhysicsSoftBodies.erase(it);
    }
    
    auto itObj = std::find(m_PhysicsObjects.begin(), m_PhysicsObjects.end(), softBody);
    if (itObj != m_PhysicsObjects.end())
    {
        m_PhysicsObjects.erase(itObj);
    }
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
    return m_PhysicsJoints.size();
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
    
    for (auto& joint : m_PhysicsJoints)
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
    return offsetof(PhysicsScene, m_Scene);
}