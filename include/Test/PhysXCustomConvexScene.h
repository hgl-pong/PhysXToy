#pragma once

#include "TestSceneBase.h"
#include <vector>

class PhysXCustomConvexScene : public TestSceneBase
{
public:
    PhysXCustomConvexScene();
    virtual ~PhysXCustomConvexScene();

    virtual void Initialize() override;
    virtual void Update(float deltaTime) override;
    virtual void Render() override;
    virtual void Cleanup() override;
    virtual void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    virtual void KeyBoardCallback(int key, int scancode, int action, int mods) override;

private:
    void CreateGround();
    
    PhysicsPtr<IPhysicsObject> CreateCylinderActor(float height, float radius, 
                                                  const MathLib::HTransform3& transform);
    
    PhysicsPtr<IPhysicsObject> CreateConeActor(float height, float radius, 
                                              const MathLib::HTransform3& transform);
    
    void ShootSphere(const MathLib::HVector3& origin, const MathLib::HVector3& direction);
    
    void PerformRaycastTests();
    
    void PerformSweepTests();
    
    void PerformOverlapTests();
    
    std::vector<PhysicsPtr<IPhysicsObject>> m_physicsObjects;
}; 