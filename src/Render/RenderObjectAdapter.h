#pragma once
#include "Renderer/Renderer.h"
#include "Physics/PhysicsCommon.h"
#include "Math/GraphicUtils/MeshData.h"

class SimpleRenderUnit;
class GizmoRenderUnit;

class RenderObjectAdapter : public RenderObject {
public:
    explicit RenderObjectAdapter(const PhysicsPtr<IPhysicsObject>& physicsObject);
    ~RenderObjectAdapter() override = default;
    
    void UpdateTransform() override;
    
    void ShowWireframe(bool show) override;
    
    void ShowBoundingBox(bool show) override;
    
    void Show(bool show) override;

    void Render(MathLib::GraphicUtils::Camera& camera) override;
    
private:
    void CreateRenderGeometry(const PhysicsPtr<IColliderGeometry>& geometry, const MathLib::HTransform3& transform);
    
    PhysicsPtr<IPhysicsObject> m_physicsObject;
    std::vector<std::shared_ptr<SimpleRenderUnit>> m_renderUnits;
    std::shared_ptr<GizmoRenderUnit> m_boundingBox;
    bool m_showWireframe = false;
    bool m_showBoundingBox = true;
    bool m_visible = true;
    bool m_isDynamic = false;
}; 