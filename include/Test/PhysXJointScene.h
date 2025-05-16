#pragma once
#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"

class PhysXJointScene : public TestSceneBase
{
public:
    PhysXJointScene();
    virtual ~PhysXJointScene();

    virtual void Initialize() override;
    virtual void Update(float deltaTime) override;
    virtual void Render() override;
    virtual void Reset() override;
    virtual void Pause() override;
    virtual void Resume() override;
    virtual void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    virtual void KeyBoardCallback(int key, int scancode, int action, int mods) override;

private:
    void CreateGround();
    void CreateLimitedSphericalChain(const MathLib::HTransform3& t, uint32_t length, const MathLib::HVector3& boxSize, MathLib::HReal separation);
    void CreateBreakableFixedChain(const MathLib::HTransform3& t, uint32_t length, const MathLib::HVector3& boxSize, MathLib::HReal separation);
    void CreateDampedD6Chain(const MathLib::HTransform3& t, uint32_t length, const MathLib::HVector3& boxSize, MathLib::HReal separation);
    
    PhysicsPtr<IPhysicsJoint> CreateLimitedSphericalJoint(PhysicsPtr<IPhysicsObject> objA, const MathLib::HTransform3& localA,
                                                         PhysicsPtr<IPhysicsObject> objB, const MathLib::HTransform3& localB);
    
    PhysicsPtr<IPhysicsJoint> CreateBreakableFixedJoint(PhysicsPtr<IPhysicsObject> objA, const MathLib::HTransform3& localA,
                                                       PhysicsPtr<IPhysicsObject> objB, const MathLib::HTransform3& localB);
    
    PhysicsPtr<IPhysicsJoint> CreateDampedD6Joint(PhysicsPtr<IPhysicsObject> objA, const MathLib::HTransform3& localA,
                                                 PhysicsPtr<IPhysicsObject> objB, const MathLib::HTransform3& localB);
}; 