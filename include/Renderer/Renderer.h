#pragma once
#include <memory>
#include <vector>
#include <functional>
#include "Physics/PhysicsCommon.h"

namespace MathLib {
    namespace GraphicUtils {
        class Camera;
    }
}

// 前向声明
class RenderObject;

/**
 * 渲染器接口类，用于替代原来的Magnum::Renderer
 */
class IRenderer {
public:
    virtual ~IRenderer() = default;

    // 释放资源
    virtual void Release() = 0;

    // 设置窗口标题
    virtual void SetApplicationName(const std::string& name) = 0;

    // 设置回调函数
    virtual void SetUp(
        std::function<void(void*)> mousePressCb,
        std::function<void(void*)> mouseReleaseCb,
        std::function<void(void*)> mouseMoveCb,
        std::function<void(void*)> mouseScrollCb,
        std::function<void(void*)> keyPressCb,
        std::function<void(void*)> keyReleaseCb
    ) = 0;

    // 添加渲染对象
    virtual void AddRenderObject(std::shared_ptr<RenderObject> renderable) = 0;
    
    // 移除渲染对象
    virtual void RemoveRenderObject(std::shared_ptr<RenderObject> renderable) = 0;
    
    // 渲染循环的单次迭代
    virtual bool Tick() = 0;
    
    // 获取当前活动的相机
    virtual MathLib::GraphicUtils::Camera* GetActiveCamera() = 0;
};

/**
 * 渲染对象接口
 */
class RenderObject {
public:
    virtual ~RenderObject() = default;
    
    // 更新变换
    virtual void UpdateTransform() = 0;
    
    // 显示或隐藏线框
    virtual void ShowWireframe(bool show) = 0;
    
    // 显示或隐藏包围盒
    virtual void ShowBoundingBox(bool show) = 0;
    
    // 显示或隐藏对象
    virtual void Show(bool show) = 0;
    
    // 渲染对象
    virtual void Render(MathLib::GraphicUtils::Camera& camera) = 0;
};

/**
 * 创建渲染器实例
 */
IRenderer* CreateRenderer(int argc, char** argv); 