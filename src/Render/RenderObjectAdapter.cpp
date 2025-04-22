#include "RenderObjectAdapter.h"
#include "Renderer/RenderUnit.h"
#include <Math/Math.h>

RenderObjectAdapter::RenderObjectAdapter(const PhysicsPtr<IPhysicsObject>& physicsObject)
    : m_physicsObject(physicsObject)
{
    if (!physicsObject) {
        return;
    }
    
    m_isDynamic = physicsObject->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
    
    std::vector<PhysicsPtr<IColliderGeometry>> geometries;
    std::vector<MathLib::HTransform3> transforms;
    physicsObject->GetColliderGeometries(geometries, &transforms);
    
    for (size_t i = 0; i < geometries.size(); i++) {
        if (geometries[i]) {
            CreateRenderGeometry(geometries[i], transforms.empty() ? MathLib::HTransform3::Identity() : transforms[i]);
        }
    }
    
    MathLib::GraphicUtils::MeshData32 boxMeshData = MathLib::GraphicUtils::GenerateBoxWireFrameMeshData<uint32_t>(MathLib::HVector3(1.0f, 1.0f, 1.0f));
    
    if (boxMeshData.m_Indices.size() % 2 != 0) {
        boxMeshData.m_Indices.pop_back(); 
    }
    
    m_boundingBox = std::make_shared<GizmoRenderUnit>(boxMeshData);
    
    float color[4] = { 0.7f, 0.7f, 0.7f, 1.0f };
    m_boundingBox->SetColor(color);
    
    MathLib::HMatrix4 identityMatrix = MathLib::HMatrix4::Identity();
    m_boundingBox->SetTransformation(&identityMatrix);
    
    UpdateTransform();
}

void RenderObjectAdapter::CreateRenderGeometry(const PhysicsPtr<IColliderGeometry>& geometry, const MathLib::HTransform3& transform)
{
    if (!geometry) {
        return;
    }

    CollisionGeometryCreateOptions options;
    geometry->GetParams(options);
    
    MathLib::GraphicUtils::MeshData32 meshData;
    MathLib::HMatrix4             scalingMatrix = MathLib::Scaling(options.m_Scale);
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
    
    std::shared_ptr<SimpleRenderUnit> renderUnit = std::make_shared<SimpleRenderUnit>(meshData);
    
    MathLib::HMatrix4 localTransform = transform.matrix();

    localTransform = localTransform * scalingMatrix;

    renderUnit->SetScale(options.m_Scale);
    renderUnit->SetTransformation(&localTransform);
    
    if (m_isDynamic) {
        float r = static_cast<float>(rand()) / RAND_MAX;
        float g = static_cast<float>(rand()) / RAND_MAX;
        float b = static_cast<float>(rand()) / RAND_MAX;
        float ambientColor[4] = { r * 0.5f, g * 0.5f, b * 0.5f, 1.0f };
        float diffuseColor[4] = { r, g, b, 1.0f };
        renderUnit->SetAmbientColor(ambientColor);
        renderUnit->SetDiffuseColor(diffuseColor);
    } else {
        float ambientColor[4] = { 0.3f, 0.1f, 0.1f, 1.0f };
        float diffuseColor[4] = { 0.8f, 0.2f, 0.2f, 1.0f };
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
    
    const MathLib::HTransform3& physTransform = m_physicsObject->GetTransform();
    MathLib::HMatrix4 renderMatrix = physTransform.matrix();
    /*
    MathLib::HMatrix4 transposed;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transposed(i, j) = physMatrix(j, i);
        }
    }
    renderMatrix = transposed;
    */
    
    for (size_t i = 0; i < m_renderUnits.size(); i++) {
        m_renderUnits[i]->SetTransformation(&renderMatrix);
    }
    
    if (m_showBoundingBox && m_boundingBox) {
        MathLib::HAABBox3D worldBBox = m_physicsObject->GetWorldBoundingBox();
        MathLib::HVector3 center = worldBBox.center();
        MathLib::HVector3 halfSize = worldBBox.sizes() / 2.0f;
        
        MathLib::HMatrix4 bboxTransform = MathLib::HMatrix4::Identity();

        bboxTransform(0, 0) = halfSize.x();
        bboxTransform(1, 1) = halfSize.y();
        bboxTransform(2, 2) = halfSize.z();

        bboxTransform(0, 3) = center.x();
        bboxTransform(1, 3) = center.y();
        bboxTransform(2, 3) = center.z();
        
        m_boundingBox->SetTransformation(&bboxTransform);
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
    if (!m_visible || !m_physicsObject) {
        return;
    }
    
    UpdateTransform();
    
    for (auto& unit : m_renderUnits) {
        if (unit) {
            unit->Render(camera);
        }
    }

    if (m_showBoundingBox && m_boundingBox) {
        m_boundingBox->Render(camera);
    }
} 