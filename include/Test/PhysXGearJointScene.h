#pragma once
#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"

struct HBoxGeometry
{
    MathLib::HVector3 extents;
};

class PhysXGearJointScene : public TestSceneBase
{
public:
    PhysXGearJointScene();
    virtual ~PhysXGearJointScene();

    virtual void Initialize() override;
    virtual void Update(float deltaTime) override;
    virtual void Render() override;
    virtual void Cleanup() override;
    virtual void Reset() override;
    virtual void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    virtual void KeyBoardCallback(int key, int scancode, int action, int mods) override;

private:
    void CreateGround();
    void CreateGearJointSetup();
    
    PhysicsPtr<IPhysicsObject> CreateGearWithBoxes(const MathLib::HTransform3& transform,
                                                const HBoxGeometry& boxGeom,
                                                int nbShapes);
    
    PhysicsPtr<IPhysicsJoint> m_HingeJoint0;
    PhysicsPtr<IPhysicsJoint> m_HingeJoint1;
    PhysicsPtr<IPhysicsJoint> m_GearJoint;
    PhysicsPtr<IPhysicsObject> m_Gear0;
    PhysicsPtr<IPhysicsObject> m_Gear1;
    
    float m_GlobalTime = 0.0f;
    bool m_AutoVelocityControl = false;
    float m_DriveVelocity = 0.5f;
}; 