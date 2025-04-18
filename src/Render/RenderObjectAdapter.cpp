#include "RenderObjectAdapter.h"
#include "Renderer/RenderUnit.h"

RenderObjectAdapter::RenderObjectAdapter(const PhysicsPtr<IPhysicsObject>& physicsObject)
    : m_physicsObject(physicsObject)
{
    if (!physicsObject) {
        return;
    }
    
    m_isDynamic = physicsObject->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    
    // 获取碰撞几何体
    std::vector<PhysicsPtr<IColliderGeometry>> geometries;
    std::vector<MathLib::HTransform3> transforms;
    physicsObject->GetColliderGeometries(geometries, &transforms);
    
    // 为每个几何体创建渲染单元
    for (size_t i = 0; i < geometries.size(); i++) {
        if (geometries[i]) {
            CreateRenderGeometry(geometries[i], transforms.empty() ? MathLib::HTransform3::Identity() : transforms[i]);
        }
    }
    
    // 创建包围盒
    MathLib::HVector3 halfSize = physicsObject->GetLocalBoundingBox().sizes() / 2.0f;
    MathLib::GraphicUtils::MeshData32 boxMeshData = MathLib::GraphicUtils::GenerateBoxWireFrameMeshData<uint32_t>(MathLib::HVector3(1, 1, 1));
    m_boundingBox = std::make_shared<GizmoRenderUnit>(boxMeshData);
    m_boundingBox->SetTransformation(&halfSize);
    
    // 设置颜色
    float color[4] = { 0.6f, 0.6f, 0.6f, 1.0f };
    m_boundingBox->SetColor(color);
}

void RenderObjectAdapter::CreateRenderGeometry(const PhysicsPtr<IColliderGeometry>& geometry, const MathLib::HTransform3& transform)
{
    CollisionGeometryCreateOptions options;
    geometry->GetParams(options);
    
    MathLib::GraphicUtils::MeshData32 meshData;
    MathLib::HVector3 scale = options.m_Scale;
    
    // 根据几何体类型创建网格数据
    switch (options.m_GeometryType) {
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
            meshData = MathLib::GraphicUtils::GenerateSphereMeshData<uint32_t>(options.m_SphereParams.m_Radius, 16, 16);
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
            meshData = MathLib::GraphicUtils::GenerateBoxMeshData<uint32_t>(options.m_BoxParams.m_HalfExtents);
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
            meshData = MathLib::GraphicUtils::GenerateCapsuleMeshData<uint32_t>(options.m_CapsuleParams.m_Radius, options.m_CapsuleParams.m_HalfHeight, 16, 16);
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
            meshData = MathLib::GraphicUtils::GeneratePlaneMeshData<uint32_t>(options.m_PlaneParams.m_Normal, options.m_PlaneParams.m_Distance);
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
            meshData.m_Vertices = options.m_TriangleMeshParams.m_Vertices;
            meshData.m_Indices = options.m_TriangleMeshParams.m_Indices;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
            meshData.m_Vertices = options.m_ConvexMeshParams.m_Vertices;
            meshData.m_Indices = options.m_ConvexMeshParams.m_Indices;
            break;
        default:
            return;
    }
    
    // 创建渲染单元
    std::shared_ptr<SimpleRenderUnit> renderUnit = std::make_shared<SimpleRenderUnit>(meshData);
    
    // 设置变换
    MathLib::HMatrix4 transformMatrix = transform.matrix();
    renderUnit->SetTransformation(&transformMatrix);
    
    // 设置颜色
    if (m_isDynamic) {
        // 动态物体使用随机色
        float r = static_cast<float>(rand()) / RAND_MAX;
        float g = static_cast<float>(rand()) / RAND_MAX;
        float b = static_cast<float>(rand()) / RAND_MAX;
        float ambientColor[4] = { r, g, b, 1.0f };
        float diffuseColor[4] = { r * 0.7f, g * 0.7f, b * 0.7f, 1.0f };
        renderUnit->SetAmbientColor(ambientColor);
        renderUnit->SetDiffuseColor(diffuseColor);
    } else {
        // 静态物体使用固定色
        float ambientColor[4] = { 0.4f, 0.0f, 0.0f, 1.0f };
        float diffuseColor[4] = { 0.7f, 0.0f, 0.0f, 1.0f };
        renderUnit->SetAmbientColor(ambientColor);
        renderUnit->SetDiffuseColor(diffuseColor);
    }
    
    m_renderUnits.push_back(renderUnit);
}

void RenderObjectAdapter::UpdateTransform()
{
    if (!m_physicsObject) {
        return;
    }
    
    const MathLib::HMatrix4& matrix = m_physicsObject->GetTransform().matrix();
    
    // 更新每个渲染单元的变换
    for (auto& unit : m_renderUnits) {
        unit->SetTransformation(&matrix);
        unit->UpdateTransformation();
    }
    
    // 更新包围盒
    if (m_showBoundingBox) {
        MathLib::HVector3 halfSize = m_physicsObject->GetWorldBoundingBox().sizes() / 2.0f;
        MathLib::HVector3 center = m_physicsObject->GetWorldBoundingBox().center();
        m_boundingBox->SetTransformation(&halfSize, &center);
        m_boundingBox->UpdateTransformation();
    }
}

void RenderObjectAdapter::ShowWireframe(bool show)
{
    m_showWireframe = show;
    for (auto& unit : m_renderUnits) {
        unit->ShowWireframe(show);
    }
}

void RenderObjectAdapter::ShowBoundingBox(bool show)
{
    m_showBoundingBox = show;
    m_boundingBox->Show(show);
}

void RenderObjectAdapter::Show(bool show)
{
    m_visible = show;
    for (auto& unit : m_renderUnits) {
        unit->Show(show);
    }
    m_boundingBox->Show(show && m_showBoundingBox);
}

void RenderObjectAdapter::Render(MathLib::GraphicUtils::Camera& camera)
{
    if (!m_visible) {
        return;
    }
    
    // 渲染每个渲染单元
    for (auto& unit : m_renderUnits) {
        unit->Render(camera);
    }
    
    // 渲染包围盒
    if (m_showBoundingBox) {
        m_boundingBox->Render(camera);
    }
} 