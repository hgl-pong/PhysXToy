#pragma once
#include "Renderer/Renderer.h"
#include "Physics/PhysicsCommon.h"
#include "Math/GraphicUtils/MeshData.h"

class SimpleRenderUnit;
class GizmoRenderUnit;

/**
 * 渲染对象适配器，用于将物理对象转换为可渲染对象
 */
class RenderObjectAdapter : public RenderObject {
public:
    explicit RenderObjectAdapter(const PhysicsPtr<IPhysicsObject>& physicsObject);
    ~RenderObjectAdapter() override = default;
    
    // 更新变换
    void UpdateTransform() override;
    
    // 显示或隐藏线框
    void ShowWireframe(bool show) override;
    
    // 显示或隐藏包围盒
    void ShowBoundingBox(bool show) override;
    
    // 显示或隐藏对象
    void Show(bool show) override;
    
    // 渲染对象
    void Render(MathLib::GraphicUtils::Camera& camera) override;
    
private:
    // 创建渲染几何体
    void CreateRenderGeometry(const PhysicsPtr<IColliderGeometry>& geometry, const MathLib::HTransform3& transform);
    
    PhysicsPtr<IPhysicsObject> m_physicsObject;
    std::vector<std::shared_ptr<SimpleRenderUnit>> m_renderUnits;
    std::shared_ptr<GizmoRenderUnit> m_boundingBox;
    bool m_showWireframe = false;
    bool m_showBoundingBox = true;
    bool m_visible = true;
    bool m_isDynamic = false;
}; 