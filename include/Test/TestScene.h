#pragma once

#include "Test/TestSceneBase.h"
#include "Physics/PhysicsCommon.h"
#include <memory>
#include <vector>

class IRenderer;

class TestScene : public TestSceneBase
{
public:
    TestScene();
    ~TestScene() override;

    void Initialize() override;
    void Update(float deltaTime) override;
    void Render() override;
    std::string GetName() const override;
    
    void Reset() override;
    void Pause() override;
    void Resume() override;
    
    void MouseClickCallback(int x, int y, int button, int action, int mods) override;
    void KeyBoardCallback(int key, int scancode, int action, int mods) override;

private:
    void CreateGround();
}; 