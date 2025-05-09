#pragma once
#include <memory>
#include <vector>
#include <functional>
#include "Physics/PhysicsCommon.h"

struct ImGuiContext;

namespace MathLib {
    namespace GraphicUtils {
        class Camera;
    }
}

class RenderObject;
class GUIPanel;
class IRenderer {
public:
    virtual ~IRenderer() = default;

    virtual void Release() = 0;
    
    virtual void SetApplicationName(const std::string& name) = 0;

    virtual void SetUp(
        std::function<void(void*)> mousePressCb,
        std::function<void(void*)> mouseReleaseCb,
        std::function<void(void*)> mouseMoveCb,
        std::function<void(void*)> mouseScrollCb,
        std::function<void(void*)> keyPressCb,
        std::function<void(void*)> keyReleaseCb
    ) = 0;

    virtual void AddRenderObject(std::shared_ptr<RenderObject> renderable) = 0;
    
    virtual void RemoveRenderObject(std::shared_ptr<RenderObject> renderable) = 0;

    virtual void AddGUIPanel(std::shared_ptr<GUIPanel> panel) = 0;

    virtual void RemoveGUIPanel(std::shared_ptr<GUIPanel> panel) = 0;
    
    virtual bool Tick() = 0;
    
    virtual MathLib::GraphicUtils::Camera* GetActiveCamera() = 0;
};

class RenderObject {
public:
    virtual ~RenderObject() = default;
    
    virtual void UpdateTransform() = 0;
    
    virtual void ShowWireframe(bool show) = 0;
    
    virtual void ShowBoundingBox(bool show) = 0;
    
    virtual void Show(bool show) = 0;
    
    virtual void Render(MathLib::GraphicUtils::Camera& camera) = 0;
};

class GUIPanel
{
public:
    virtual void Render() = 0;
    virtual void SetVisible(bool visible) = 0;
    virtual bool IsVisible() const = 0;
};

// Forward declaration of renderer creation function
IRenderer* CreateRenderer(int argc, char** argv); 