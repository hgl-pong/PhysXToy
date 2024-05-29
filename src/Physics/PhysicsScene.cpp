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
    const uint32_t offset = physicsObject->GetOffset();
    bool result = false;
    swtich(physicsObject->GetType())
    {
    case PhysicsObjectType::eRIGID_STATIC:
    {
        const PxRigidStatic *pRigidStatic = reinterpret_cast<PxRigidStatic *>(reinterpret_cast<void *>(physicsObject) + offset);
        m_Scene->addActor(*pRigidStatic);
        result = m_RigidStatic.emplace(dynamic_cast<RigidStatic *>(physicsObject)).second;
        break;
    }
    case PhysicsObjectType::eRIGID_DYNAMIC:
    {
        const PxRigidDynamic *pRigidDynamic = reinterpret_cast<PxRigidDynamic *>(reinterpret_cast<void *>(physicsObject) + offset);
        m_Scene->addActor(*pRigidDynamic);
        result = m_RigidDynamic.emplace(dynamic_cast<RigidDynamic *>(physicsObject)).second;
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
    m_PhysicsObjects.erase(physicsObject);
}

uint32_t PhysicsScene::GetPhysicsObjectCount() const
{
    return m_PhysicsObjects.size();
}

uint32_t PhysicsScene::GetOffset() const
{
    return PX_OFFSET_OF(this, m_Scene.get());
}