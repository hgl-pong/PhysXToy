#pragma once
#include <string>
#include <memory>
#include "../Renderer/Renderer.h"

enum TestSceneType : uint32_t
{
    DEFAULT_SCENE,
    PHYSX_HELLO_WORLD,
    PHYSX_MASS_PROPERTIES,
    PHYSX_JOINT_SCENE,
    PHYSX_SPLIT_SIM_SCENE,
    PHYSX_GYROSCOPIC_SCENE,
    PHYSX_RACK_JOINT_SCENE,
    PHYSX_CUSTOM_CONVEX_SCENE,
    PHYSX_GEAR_JOINT_SCENE,
    PHYSX_JOINT_DRIVE_SCENE,
    TEST_SCENE_COUNT
};

static std::string testSceneName[] =
{
    "Default Scene",
    "PhysX Hello World",
    "PhysX Mass Properties",
    "PhysX Joint Scene",
    "PhysX Split Sim Scene",
    "PhysX Gyroscopic Scene",
    "PhysX Rack Joint Scene",
    "PhysX Custom Convex Scene",
    "PhysX Gear Joint Scene",
    "PhysX Joint Drive Scene",
};

static std::string testSceneDesc[] = 
{
    "Default physics scene with ground plane and physics objects",
    "Basic physics scene with a ground plane, sphere, and boxes. As Same as Hello World in PhysX Snippet.",
    "Show different ways to set the rigid body mass, create 5 snowmen with different mass properties",
    "PhysX Joint Scene, show different types of joints: limited spherical joint, breakable fixed joint, and damped D6 joint.",
    "PhysX Split Simulation Scene, demonstrates how to overlap collision detection with rendering and application work. Press M to toggle between no-lag and one-frame-lag modes.",
    "PhysX Gyroscopic Scene, demonstrates the Dzhanibekov effect by enabling gyroscopic forces on rotating objects. Press SPACE to toggle gyroscopic forces on/off.",
    "PhysX Rack Joint Scene, demonstrates a rack and pinion joint mechanism where a rotating gear drives a linear rack. Press SPACE to toggle the drive direction.",
    "PhysX Custom Convex Scene, demonstrates the use of custom convex geometries (cylinders and cones) using PhysicsLib interfaces. Press SPACE to shoot spheres.",
    "PhysX Gear Joint Scene, demonstrates a gear joint mechanism where two rotating gears are connected with a gear joint. Press SPACE to toggle automatic velocity control.",
    "PhysX Joint Drive Scene, demonstrates D6 joint drives with different modes: Linear X, Angular Twist, Angular Swing, and Angular SLERP. Press keys 1-4 to switch drive modes.",
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
