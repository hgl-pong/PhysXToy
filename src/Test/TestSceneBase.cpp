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

void TestSceneBase::Cleanup()
{
    for (auto& object : m_TestObjects)
    {
        if (m_Renderer)
        {
            m_Renderer->RemoveRenderObject(object.renderObject);
        }
        if (m_Scene)
            m_Scene->RemovePhysicsObject(object.physicsObject);
    }

    m_TestObjects.clear();

    m_Scene.reset();
    m_Material.reset();

    m_initialized = false;
}

void TestSceneBase::AddObject(PhysicsPtr<IPhysicsObject> &object, bool createRenderObject)
{
    TestObject newObject;
    newObject.physicsObject = object;
    if (createRenderObject && m_Renderer)
    {
        std::shared_ptr<RenderObject> renderable = std::make_shared<RenderObjectAdapter>(object);
        newObject.renderObject = renderable;
        m_Renderer->AddRenderObject(renderable);
    }
    m_TestObjects.push_back(newObject);
    if (m_Scene)
        m_Scene->AddPhysicsObject(object);
} 

void TestSceneBase::RemoveObject(PhysicsPtr<IPhysicsObject>& object)
{
    auto it = std::find_if(m_TestObjects.begin(), m_TestObjects.end(),
        [&object](const TestObject& obj) { return obj.physicsObject == object; });
    if (it != m_TestObjects.end())
    {
        if (m_Renderer && it->renderObject)
        {
            m_Renderer->RemoveRenderObject(it->renderObject);
        }
        std::swap(*it, m_TestObjects.back());
        m_TestObjects.pop_back();
    }

    if (m_Scene)
        m_Scene->RemovePhysicsObject(object);
}
