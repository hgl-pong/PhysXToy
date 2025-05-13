#pragma once

#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"
#include <memory>
#include <vector>

// 前向声明
class IRenderer;

class TestScene : public TestSceneBase
{
public:
    TestScene();
    ~TestScene() override;

    void Initialize() override;
    void Update(float deltaTime) override;
    void Render() override;
    void Cleanup() override;
    std::string GetName() const override;
    
    void Reset() override;
    void Pause() override;
    void Resume() override;
    
    void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    void KeyBoardCallback(int key, int scancode, int action, int mods) override;
    
    void SetRenderer(PhysicsPtr<IRenderer> renderer) { m_Renderer = renderer; }
    PhysicsPtr<IRenderer> GetRenderer() const { return m_Renderer; }

private:
    void CreateGround();
    PhysicsPtr<IPhysicsObject> CreateDynamic(const MathLib::HTransform3 &t, 
                                            PhysicsPtr<IColliderGeometry> &geometry, 
                                            const MathLib::HVector3 &velocity = MathLib::HVector3(0, 0, 0));
    
    void AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject> &object);

private:
    PhysicsPtr<IRenderer> m_Renderer;
    PhysicsPtr<IPhysicsMaterial> m_Material;
    PhysicsPtr<IPhysicsScene> m_Scene;
    std::vector<PhysicsPtr<IPhysicsObject>> m_PhysicsObjects;
}; 