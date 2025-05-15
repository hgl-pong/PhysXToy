#pragma once
#include <string>
#include <memory>
#include "../Renderer/Renderer.h"

enum TestSceneType : uint32_t
{
    DEFAULT_SCENE,
    PHYSX_HELLO_WORLD,
    PHYSX_MASS_PROPERTIES,
    TEST_SCENE_COUNT
};

static std::string testSceneName[] =
{
    "Default Scene",
    "PhysX Hello World",
    "PhysX Mass Properties",
};

static std::string testSceneDesc[] = 
{
    "Default physics scene with ground plane and physics objects",
    "Basic physics scene with a ground plane, sphere, and boxes. As Same as Hello World in PhysX Snippet.",
    "Show different ways to set the rigid body mass, create 5 snowmen with different mass properties",
};

class TestSceneBase
{
public:
    TestSceneBase(TestSceneType type);
    virtual ~TestSceneBase();
    virtual void Initialize() = 0;
    virtual void Update(float deltaTime) = 0;
    virtual void Render() = 0;
    virtual void Cleanup();

    std::string GetName() const { return testSceneName[m_SceneType]; }
    virtual std::string GetDescription() const
    {
        return testSceneDesc[m_SceneType];
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
    void AddObject(PhysicsPtr<IPhysicsObject> &object, bool createRenderObject = true);
    void RemoveObject(PhysicsPtr<IPhysicsObject> &object);

protected:
    bool m_initialized = false;
    bool m_paused = false;
    float m_elapsedTime = 0.0f;
    PhysicsPtr<IPhysicsScene> m_Scene;
    PhysicsPtr<IPhysicsMaterial> m_Material;
    struct TestObject
    {
        PhysicsPtr<IPhysicsObject> physicsObject;
        std::shared_ptr<RenderObject> renderObject;
    };
    TestSceneType m_SceneType;
    std::vector<TestObject> m_TestObjects;
};
