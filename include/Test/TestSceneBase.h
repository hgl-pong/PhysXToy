#pragma once

#include <string>
#include <memory>

enum class TestSceneType
{
    PHYSX_HELLO_WORLD,
};

class TestSceneBase
{
public:
    TestSceneBase();
    virtual ~TestSceneBase();

    virtual void Initialize() = 0;
    virtual void Update(float deltaTime) = 0;
    virtual void Render() = 0;
    virtual void Cleanup() = 0;

    virtual std::string GetName() const = 0;
    virtual void Reset();
    virtual void Pause();
    virtual void Resume();

    virtual void MouseClickCallback(int x, int y, int button, int action, int mods) = 0;
    virtual void KeyBoardCallback(int key, int scancode, int action, int mods) = 0;

    bool IsInitialized() const { return m_initialized; }
    bool IsPaused() const { return m_paused; }

    // 工厂方法，创建指定类型的场景
    static std::shared_ptr<TestSceneBase> CreateScene(TestSceneType type);

protected:
    bool m_initialized = false;
    bool m_paused = false;
    float m_elapsedTime = 0.0f;
};
