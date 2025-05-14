#include "Test/TestSceneBase.h"
#include <string>
#include <memory>
#include "../Render/RenderObjectAdapter.h"
#include "TestRigidBodyCreate.h"


TestSceneBase::TestSceneBase(std::string description)
        : m_initialized(false)
    , m_paused(false)
    , m_elapsedTime(0.0f)
    , m_description(description)
{
    m_Renderer = std::shared_ptr<IRenderer>(GetRenderer());
}

TestSceneBase::~TestSceneBase()
{
}

PhysicsPtr<IPhysicsObject> TestSceneBase::CreateDynamic(PhysicsPtr<IPhysicsScene>& scene, const MathLib::HTransform3& t,
        PhysicsPtr<IColliderGeometry>& geometry,
        const MathLib::HVector3& velocity)
{
    auto dynamic = TestRigidBody::CreateDynamic(t, geometry, velocity);
    if (m_Scene)
    {
        m_Scene->AddPhysicsObject(dynamic);
        AddPhysicsDebugRenderableObject(dynamic);
        m_PhysicsObjects.push_back(dynamic);
    }
    return dynamic;
}

void TestSceneBase::AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject> &object)
{
    if (m_Renderer)
    {
        std::shared_ptr<RenderObject> renderable = std::make_shared<RenderObjectAdapter>(object);
        m_Renderer->AddRenderObject(renderable);
    }
} 