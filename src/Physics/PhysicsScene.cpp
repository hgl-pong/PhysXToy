#include "Physics/PhysicsScene.h"
#include "PxPhysicsAPI.h"
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
    m_Scene->release();
    m_Scene.reset();
}

bool PhysicsScene::AddPhysicsObject(IPhysicsObject *physicsObject)
{
    // m_Scene->addActor(physicsObject->GetPhysicsObject());
    auto it = m_PhysicsObjects.emplace(physicsObject);
    return it.second;
}

void PhysicsScene::RemovePhysicsObject(IPhysicsObject *physicsObject)
{
    // m_Scene->removeActor(physicsObject->GetPhysicsObject());
    m_PhysicsObjects.erase(physicsObject);
}

uint32_t PhysicsScene::GetPhysicsObjectCount() const
{
    return m_PhysicsObjects.size();
}