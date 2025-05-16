#pragma once
#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"

class PhysXRackJointScene : public TestSceneBase
{
public:
    PhysXRackJointScene();
    virtual ~PhysXRackJointScene();

    virtual void Initialize() override;
    virtual void Update(float deltaTime) override;
    virtual void Render() override;
    virtual void Cleanup() override;
    virtual void Reset() override;
    virtual void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    virtual void KeyBoardCallback(int key, int scancode, int action, int mods) override;

private:
    void CreateGround();
    void CreateRackAndPinionSetup();
    
    PhysicsPtr<IPhysicsObject> CreateGearWithBoxes(const MathLib::HTransform3& transform,
                                                 const MathLib::HVector3& boxSize,
                                                 int nbShapes);
    
    // 创建具有箱体几何形状的齿条
    PhysicsPtr<IPhysicsObject> CreateRackWithBoxes(const MathLib::HTransform3& transform,
                                                 int nbTeeth,
                                                 float rackLength);
    
    PhysicsPtr<IPhysicsJoint> m_HingeJoint; 
    PhysicsPtr<IPhysicsJoint> m_PrismaticJoint;  
    PhysicsPtr<IPhysicsJoint> m_RackJoint;  
    PhysicsPtr<IPhysicsObject> m_Gear;  
    PhysicsPtr<IPhysicsObject> m_Rack;  
    
    float m_GlobalTime = 0.0f;
    bool m_DriveDirectionPositive = true;
    float m_GearDriveVelocity = 3.0f;
}; 