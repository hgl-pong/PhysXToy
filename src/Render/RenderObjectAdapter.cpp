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
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD:
            {
                const auto& hfParams = options.m_HeightFieldParams;
                const int rows = hfParams.m_Rows;
                const int cols = hfParams.m_Columns;
                const float rowScale = hfParams.m_RowScale;
                const float colScale = hfParams.m_ColumnScale;
                const float heightScale = hfParams.m_HeightScale;
                
                meshData.m_Vertices.resize(rows * cols);
                meshData.m_Indices.reserve((rows-1) * (cols-1) * 6);
                
                for (int r = 0; r < rows; r++) {
                    for (int c = 0; c < cols; c++) {
                        float x = (c - cols/2.0f) * colScale;
                        float z = (r - rows/2.0f) * rowScale;
                        float y = hfParams.m_HeightData[r * cols + c] * heightScale;
                        
                        meshData.m_Vertices[r * cols + c] = MathLib::HVector3(x, y, z);
                    }
                }
                
                for (int r = 0; r < rows-1; r++) {
                    for (int c = 0; c < cols-1; c++) {
                        uint32_t i00 = r * cols + c;
                        uint32_t i10 = r * cols + (c+1);
                        uint32_t i01 = (r+1) * cols + c;
                        uint32_t i11 = (r+1) * cols + (c+1);
                        
                        meshData.m_Indices.push_back(i00);
                        meshData.m_Indices.push_back(i01);
                        meshData.m_Indices.push_back(i10);
                        
                        meshData.m_Indices.push_back(i10);
                        meshData.m_Indices.push_back(i01);
                        meshData.m_Indices.push_back(i11);
                    }
                }
            }
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
        float specularParams[4] = { 0.7f, 64.0f, 0.4f, 0.8f };
        float lightParams[4] = { 1.0f, 0.09f, 0.032f, 0.15f };
        renderUnit->SetAmbientColor(ambientColor);
        renderUnit->SetDiffuseColor(diffuseColor);
        renderUnit->SetSpecularParams(specularParams);
        renderUnit->SetLightParams(lightParams);
    } else {
        float ambientColor[4] = { 0.3f, 0.1f, 0.1f, 1.0f };
        float diffuseColor[4] = { 0.8f, 0.2f, 0.2f, 1.0f };
        float specularParams[4] = { 0.3f, 16.0f, 0.2f, 0.7f };
        float lightParams[4] = { 1.0f, 0.09f, 0.032f, 0.1f };
        renderUnit->SetAmbientColor(ambientColor);
        renderUnit->SetDiffuseColor(diffuseColor);
        renderUnit->SetSpecularParams(specularParams);
        renderUnit->SetLightParams(lightParams);
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