#pragma once
#include "Test/TestSceneBase.h"
#include <vector>

// Split simulation scene demonstrates how to use PhysX split simulation feature
// to overlap collision detection with rendering and application work
class PhysXSplitSimScene : public TestSceneBase
{
public:
    PhysXSplitSimScene();
    ~PhysXSplitSimScene();

    void Initialize() override;
    void Update(float deltaTime) override;
    void Render() override;
    void Cleanup() override;

    void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    void KeyBoardCallback(int key, int scancode, int action, int mods) override;

    void Reset() override;

private:
    void CreateGround();
    void CreateKinematics();
    void CreateDynamics();
    void UpdateKinematicTargets(float deltaTime);
    void ApplyKinematicTargets();

    static const int NB_KINE_X = 16;
    static const int NB_KINE_Y = 16;
    static constexpr float KINE_SCALE = 3.1f;

    // Store kinematic actors for animation
    PhysicsPtr<IPhysicsObject> m_Kinematics[NB_KINE_Y][NB_KINE_X];
    MathLib::HTransform3 m_KinematicTargets[NB_KINE_Y][NB_KINE_X];

    // Simulation state
    bool m_isFirstFrame = true;
    bool m_useNoLagMode = true; // true: no lag mode, false: one frame lag mode
    float m_time = 0.0f;
}; 