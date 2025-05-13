#pragma once
#include <string>
#include <memory>

enum TestSceneType : uint32_t
{
    DEFAULT_SCENE,
    PHYSX_HELLO_WORLD,
    TEST_SCENE_COUNT
};

static std::string testSceneName[] =
{
    "Default Scene",
    "PhysX Hello World",
};

class TestSceneBase
{
public:
    TestSceneBase(std::string description)
        : m_initialized(false)
    , m_paused(false)
    , m_elapsedTime(0.0f)
    , m_description(description)
    {
    }
    virtual ~TestSceneBase()
    {

    }

    virtual void Initialize() = 0;
    virtual void Update(float deltaTime) = 0;
    virtual void Render() = 0;
    virtual void Cleanup() = 0;

    virtual std::string GetName() const = 0;
    virtual std::string GetDescription() const
    {
        return m_description;
    }

    virtual void Reset()
    {
        m_initialized = false;
        m_paused = false;
        m_elapsedTime = 0.0f;
    }

    virtual void Pause()
    {
        m_paused = true;
    }

    virtual void Resume()
    {
        m_paused = false;
    }

    virtual void MouseClickCallback(int x, int y, int button, int action, int mods) = 0;
    virtual void KeyBoardCallback(int key, int scancode, int action, int mods) = 0;

    bool IsInitialized() const { return m_initialized; }
    bool IsPaused() const { return m_paused; }

protected:
    bool m_initialized = false;
    bool m_paused = false;
    float m_elapsedTime = 0.0f;
    std::string m_description;
};
