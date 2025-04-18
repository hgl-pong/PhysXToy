#pragma once
#include <memory>
#include <vector>
#include "Renderer/Renderer.h"
#include "Math/GraphicUtils/MeshData.h"

/**
 * 基础渲染单元类
 */
class BaseRenderUnit {
public:
    virtual ~BaseRenderUnit() = default;
    
    // 设置变换
    virtual void SetTransformation(const MathLib::HMatrix4* transform = nullptr) = 0;
    
    // 设置变换（带缩放和偏移）
    virtual void SetTransformation(const MathLib::HVector3* scale = nullptr, const MathLib::HVector3* position = nullptr) = 0;
    
    // 更新变换
    virtual void UpdateTransformation() = 0;
    
    // 显示或隐藏对象
    virtual void Show(bool show) = 0;
    
    // 渲染对象
    virtual void Render(MathLib::GraphicUtils::Camera& camera) = 0;
    
    // 添加到场景
    virtual void AddToScene(void* scene) = 0;
    
    // 从场景中移除
    virtual void RemoveFromScene() = 0;
};

/**
 * 简单渲染单元类，用于渲染基本几何体
 */
class SimpleRenderUnit : public BaseRenderUnit {
public:
    explicit SimpleRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData);
    ~SimpleRenderUnit() override;
    
    void SetTransformation(const MathLib::HMatrix4* transform = nullptr) override;
    void SetTransformation(const MathLib::HVector3* scale = nullptr, const MathLib::HVector3* position = nullptr) override;
    void UpdateTransformation() override;
    void Show(bool show) override;
    void Render(MathLib::GraphicUtils::Camera& camera) override;
    void AddToScene(void* scene) override;
    void RemoveFromScene() override;
    
    // 显示或隐藏线框
    void ShowWireframe(bool show);
    
    // 设置环境色
    void SetAmbientColor(const float* color);
    
    // 设置漫反射色
    void SetDiffuseColor(const float* color);
    
    // 获取环境色
    const float* GetAmbientColor() const;
    
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

/**
 * 线框渲染单元类，用于渲染线框
 */
class GizmoRenderUnit : public BaseRenderUnit {
public:
    explicit GizmoRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData);
    ~GizmoRenderUnit() override;
    
    void SetTransformation(const MathLib::HMatrix4* transform = nullptr) override;
    void SetTransformation(const MathLib::HVector3* scale = nullptr, const MathLib::HVector3* position = nullptr) override;
    void UpdateTransformation() override;
    void Show(bool show) override;
    void Render(MathLib::GraphicUtils::Camera& camera) override;
    void AddToScene(void* scene) override;
    void RemoveFromScene() override;
    
    // 设置颜色
    void SetColor(const float* color);
    
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
}; 