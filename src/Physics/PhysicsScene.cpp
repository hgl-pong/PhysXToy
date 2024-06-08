#include "PhysicsScene.h"
#include "PxPhysicsAPI.h"
#include "PhysicsObject.h"
#include "Utility/PhysXUtils.h"
#ifndef NDEBUG
#define ENABLE_PVD
#endif
using namespace physx;

PhysicsScene::PhysicsScene(const PhysicsSceneCreateOptions &options, physx::PxCpuDispatcher *cpuDispatch)
{
    auto &physics = PxGetPhysics();
    PxSceneDesc sceneDesc(physics.getTolerancesScale());
    sceneDesc.gravity = PxVec3(options.m_Gravity[0], options.m_Gravity[1], options.m_Gravity[2]);
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

    for (auto &dynamicObject : m_RigidDynamic)
    {
        dynamicObject->Update();
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
        PxRigidStatic *pRigidStatic = reinterpret_cast<PhysXPtr<PxRigidStatic> *>(reinterpret_cast<char *>(physicsObject.get()) + offset)->get();
        if (pRigidStatic->getNbShapes() == 0)
            return false;
        if (m_Scene->addActor(*pRigidStatic))
            result = m_RigidStatic.emplace(physicsObject).second;
        if (!result)
            m_Scene->removeActor(*pRigidStatic);
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
    {
        PxRigidDynamic *pRigidDynamic = reinterpret_cast<PhysXPtr<PxRigidDynamic> *>(reinterpret_cast<char *>(physicsObject.get()) + offset)->get();
        if (pRigidDynamic->getNbShapes() == 0)
            return false;
        if (m_Scene->addActor(*pRigidDynamic))
            result = m_RigidDynamic.emplace(physicsObject).second;
        if (!result)
            m_Scene->removeActor(*pRigidDynamic);
        break;
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
        m_RigidStatic.erase(physicsObject);
        break;
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
        m_RigidDynamic.erase(physicsObject);
        break;
    default:
        break;
    }
}

uint32_t PhysicsScene::GetPhysicsObjectCount() const
{
    return m_RigidDynamic.size() + m_RigidStatic.size();
}

uint32_t PhysicsScene::GetPhysicsRigidDynamicCount() const
{
    return m_RigidDynamic.size();
}

uint32_t PhysicsScene::GetPhysicsRigidStaticCount() const
{
    return m_RigidStatic.size();
}

size_t PhysicsScene::GetOffset() const
{
    return offsetof(PhysicsScene, m_Scene);
}