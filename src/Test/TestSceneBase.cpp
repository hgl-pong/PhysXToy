#include "Test/TestSceneBase.h"
#include <string>
#include <memory>
#include "../Render/RenderObjectAdapter.h"
#include "TestRigidBodyCreate.h"


TestSceneBase::TestSceneBase(TestSceneType type)
        : m_initialized(false)
    , m_paused(false)
    , m_elapsedTime(0.0f)
    , m_SceneType(type)
{
}

TestSceneBase::~TestSceneBase()
{
    Cleanup();
    PhysicsEngineUtils::GetPhysicsEngine()->SetActiveScene(nullptr);
}

void TestSceneBase::Cleanup()
{
    auto renderer = GetRenderer();

    for (auto& object : m_TestObjects)
    {
        if (renderer && object.renderObject)
        {
            renderer->RemoveRenderObject(object.renderObject);
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
    auto renderer = GetRenderer();
    if (createRenderObject && renderer)
    {
        std::shared_ptr<RenderObject> renderable = std::make_shared<RenderObjectAdapter>(object);
        newObject.renderObject = renderable;
        renderer->AddRenderObject(renderable);
    }
    m_TestObjects.push_back(newObject);
    if (m_Scene)
        m_Scene->AddPhysicsObject(object);
} 

void TestSceneBase::RemoveObject(PhysicsPtr<IPhysicsObject>& object)
{
    auto renderer = GetRenderer();
    auto it = std::find_if(m_TestObjects.begin(), m_TestObjects.end(),
        [&object](const TestObject& obj) { return obj.physicsObject == object; });
    if (it != m_TestObjects.end())
    {
        if (renderer && it->renderObject)
        {
            renderer->RemoveRenderObject(it->renderObject);
        }
        std::swap(*it, m_TestObjects.back());
        m_TestObjects.pop_back();
    }

    if (m_Scene)
        m_Scene->RemovePhysicsObject(object);
}
