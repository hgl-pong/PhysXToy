#pragma once

#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"
#include <memory>
#include <vector>

class PhysXJointDriveScene : public TestSceneBase
{
public:
    PhysXJointDriveScene();
    ~PhysXJointDriveScene() override;

    void Initialize() override;
    void Update(float deltaTime) override;
    void Render() override;
    void Cleanup() override;

    void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    void KeyBoardCallback(int key, int scancode, int action, int mods) override;

    void Reset() override;

private:
    void CreateGround();
    void CreateScene();
    void CreateD6JointWithDrive(uint32_t sceneIndex);
    
    // Scene parameters
    bool m_changeObjectAType = false;        // Toggle between static and kinematic for object A
    bool m_changeObjectBRotation = false;    // Toggle object B rotation
    bool m_changeJointFrameARotation = false; // Toggle joint frame A rotation
    bool m_changeJointFrameBRotation = false; // Toggle joint frame B rotation
    uint32_t m_sceneIndex = 0;               // Current drive scene index
    static constexpr uint32_t MAX_SCENE_INDEX = 4; // Number of different drive types
    
    // Physics objects
    PhysicsPtr<IPhysicsObject> m_objectA;
    PhysicsPtr<IPhysicsObject> m_objectB;
    PhysicsPtr<IPhysicsJoint> m_d6Joint;
}; 