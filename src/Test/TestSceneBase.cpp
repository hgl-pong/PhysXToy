#include "Test/TestSceneBase.h"
#include "Test/PhysXHelloWorldScene.h"

TestSceneBase::TestSceneBase()
    : m_initialized(false)
    , m_paused(false)
    , m_elapsedTime(0.0f)
{
}

TestSceneBase::~TestSceneBase()
{
}

void TestSceneBase::Reset()
{
    m_initialized = false;
    m_paused = false;
    m_elapsedTime = 0.0f;
}

void TestSceneBase::Pause()
{
    m_paused = true;
}

void TestSceneBase::Resume()
{
    m_paused = false;
}

std::shared_ptr<TestSceneBase> TestSceneBase::CreateScene(TestSceneType type)
{
    switch (type)
    {
    case TestSceneType::PHYSX_HELLO_WORLD:
        return std::make_shared<PhysXHelloWorldScene>();
    default:
        return nullptr;
    }
} 