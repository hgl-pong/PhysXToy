#pragma once
#include "Test/TestSceneBase.h"
#include <vector>

// PhysXGyroscopicScene demonstrates the Dzhanibekov effect
// by enabling gyroscopic forces on rigid bodies
class PhysXGyroscopicScene : public TestSceneBase
{
public:
    PhysXGyroscopicScene();
    ~PhysXGyroscopicScene();

    void Initialize() override;
    void Update(float deltaTime) override;
    void Render() override;
    void Cleanup() override;

    void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    void KeyBoardCallback(int key, int scancode, int action, int mods) override;

    void Reset() override;

private:
    void CreateTumblingObject();
    
    bool m_gyroscopicForcesEnabled = true;
    PhysicsPtr<IPhysicsObject> m_tumblingObject;
}; 