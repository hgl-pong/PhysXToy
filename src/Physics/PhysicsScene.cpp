#include "Physics/PhysicsScene.h"
#include "PxPhysicsAPI.h"
#include "Physics/PhysicsObject.h"
using namespace physx;

PhysicsScene::PhysicsScene()
{
}

PhysicsScene::~PhysicsScene()
{
}

void PhysicsScene::Tick(float deltaTime)
{
    m_Scene->simulate(1.0f / 60.0f);
    m_Scene->fetchResults(true);
}

void PhysicsScene::Init()
{
#ifdef ENABLE_PVD
    PxPvdSceneClient *pvdClient = m_Scene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }
#endif
}

void PhysicsScene::UnInit()
{
    m_Scene.reset();
}

bool PhysicsScene::AddPhysicsObject(IPhysicsObject *physicsObject)
{
    // m_Scene->addActor(physicsObject->GetPhysicsObject());
    const uint32_t offset = physicsObject->GetOffset();
    bool result = false;
    switch(physicsObject->GetType())
    {
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
    {
        PxRigidStatic *pRigidStatic = *reinterpret_cast<PxRigidStatic **>(reinterpret_cast<char *>(physicsObject) + offset);
        if(m_Scene->addActor(*pRigidStatic))
            result = m_RigidStatic.emplace(dynamic_cast<PhysicsRigidStatic *>(physicsObject)).second;
        if(!result)
            m_Scene->removeActor(*pRigidStatic);
        break;
    }
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
    {
        PxRigidDynamic *pRigidDynamic = *reinterpret_cast<PxRigidDynamic **>(reinterpret_cast<char *>(physicsObject) + offset);
        if(m_Scene->addActor(*pRigidDynamic))
            result = m_RigidDynamic.emplace(dynamic_cast<PhysicsRigidDynamic *>(physicsObject)).second;
        if (!result)
            m_Scene->removeActor(*pRigidDynamic);
        break;
    }
    default:
        break;
    }
    return result;
}

void PhysicsScene::RemovePhysicsObject(IPhysicsObject *physicsObject)
{
    // m_Scene->removeActor(physicsObject->GetPhysicsObject());
    switch (physicsObject->GetType())
    {
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC:
        m_RigidStatic.erase(dynamic_cast<PhysicsRigidStatic*>(physicsObject));
        break;
    case PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC:
        m_RigidDynamic.erase(dynamic_cast<PhysicsRigidDynamic*>(physicsObject));
        break;
    default:
        break;
    }
}

uint32_t PhysicsScene::GetPhysicsObjectCount() const
{
    return m_RigidDynamic.size()+m_RigidStatic.size();
}

size_t PhysicsScene::GetOffset() const
{
    return offsetof(PhysicsScene, m_Scene);
}