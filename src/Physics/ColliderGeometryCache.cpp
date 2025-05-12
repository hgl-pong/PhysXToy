#include "ColliderGeometry.h"
#include "Utility/PhysicsUtils.h"
#include <functional>

bool AreGeometriesEqual(const CollisionGeometryCreateOptions& a, const CollisionGeometryCreateOptions& b)
{
    if (a.m_GeometryType != b.m_GeometryType)
        return false;
    
    if (!(MathLib::Equal(a.m_Scale[0], b.m_Scale[0]) && 
          MathLib::Equal(a.m_Scale[1], b.m_Scale[1]) && 
          MathLib::Equal(a.m_Scale[2], b.m_Scale[2])))
        return false;
    
    switch (a.m_GeometryType)
    {
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
        {
            const auto& aHalfExtents = a.m_BoxParams.m_HalfExtents;
            const auto& bHalfExtents = b.m_BoxParams.m_HalfExtents;
            
            return MathLib::Equal(aHalfExtents[0], bHalfExtents[0]) && 
                   MathLib::Equal(aHalfExtents[1], bHalfExtents[1]) && 
                   MathLib::Equal(aHalfExtents[2], bHalfExtents[2]);
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
        {
            return MathLib::Equal(a.m_SphereParams.m_Radius, b.m_SphereParams.m_Radius);
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
        {
            return MathLib::Equal(a.m_CapsuleParams.m_Radius, b.m_CapsuleParams.m_Radius) && 
                   MathLib::Equal(a.m_CapsuleParams.m_HalfHeight, b.m_CapsuleParams.m_HalfHeight);
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
        {
            const auto& aNormal = a.m_PlaneParams.m_Normal;
            const auto& bNormal = b.m_PlaneParams.m_Normal;
            
            return MathLib::Equal(aNormal[0], bNormal[0]) && 
                   MathLib::Equal(aNormal[1], bNormal[1]) && 
                   MathLib::Equal(aNormal[2], bNormal[2]) && 
                   MathLib::Equal(a.m_PlaneParams.m_Distance, b.m_PlaneParams.m_Distance);
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
        {
            const auto& aVertices = a.m_TriangleMeshParams.m_Vertices;
            const auto& bVertices = b.m_TriangleMeshParams.m_Vertices;
            const auto& aIndices = a.m_TriangleMeshParams.m_Indices;
            const auto& bIndices = b.m_TriangleMeshParams.m_Indices;
            
            if (aVertices.size() != bVertices.size() || aIndices.size() != bIndices.size())
                return false;
            
            const size_t vertexSampleCount = std::min<size_t>(10, aVertices.size());
            const size_t indexSampleCount = std::min<size_t>(30, aIndices.size());
            
            for (size_t i = 0; i < vertexSampleCount; ++i)
            {
                size_t index = (i * aVertices.size()) / vertexSampleCount;
                const auto& aVert = aVertices[index];
                const auto& bVert = bVertices[index];
                
                if (!MathLib::Equal(aVert[0], bVert[0]) || 
                    !MathLib::Equal(aVert[1], bVert[1]) || 
                    !MathLib::Equal(aVert[2], bVert[2]))
                    return false;
            }
            
            for (size_t i = 0; i < indexSampleCount; ++i)
            {
                size_t index = (i * aIndices.size()) / indexSampleCount;
                if (aIndices[index] != bIndices[index])
                    return false;
            }
            
            return true;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
        {
            const auto& aVertices = a.m_ConvexMeshParams.m_Vertices;
            const auto& bVertices = b.m_ConvexMeshParams.m_Vertices;
            const auto& aIndices = a.m_ConvexMeshParams.m_Indices;
            const auto& bIndices = b.m_ConvexMeshParams.m_Indices;
            
            if (aVertices.size() != bVertices.size() || aIndices.size() != bIndices.size())
                return false;
            
            const size_t vertexSampleCount = std::min<size_t>(10, aVertices.size());
            const size_t indexSampleCount = std::min<size_t>(30, aIndices.size());
            
            for (size_t i = 0; i < vertexSampleCount; ++i)
            {
                size_t index = (i * aVertices.size()) / vertexSampleCount;
                const auto& aVert = aVertices[index];
                const auto& bVert = bVertices[index];
                
                if (!MathLib::Equal(aVert[0], bVert[0]) || 
                    !MathLib::Equal(aVert[1], bVert[1]) || 
                    !MathLib::Equal(aVert[2], bVert[2]))
                    return false;
            }
            
            for (size_t i = 0; i < indexSampleCount; ++i)
            {
                size_t index = (i * aIndices.size()) / indexSampleCount;
                if (aIndices[index] != bIndices[index])
                    return false;
            }
            
            return true;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD:
        {
            const auto& aHeightData = a.m_HeightFieldParams.m_HeightData;
            const auto& bHeightData = b.m_HeightFieldParams.m_HeightData;
            
            if (a.m_HeightFieldParams.m_Rows != b.m_HeightFieldParams.m_Rows ||
                a.m_HeightFieldParams.m_Columns != b.m_HeightFieldParams.m_Columns ||
                !MathLib::Equal(a.m_HeightFieldParams.m_RowScale, b.m_HeightFieldParams.m_RowScale) ||
                !MathLib::Equal(a.m_HeightFieldParams.m_ColumnScale, b.m_HeightFieldParams.m_ColumnScale) ||
                !MathLib::Equal(a.m_HeightFieldParams.m_HeightScale, b.m_HeightFieldParams.m_HeightScale))
                return false;
            
            if (aHeightData.size() != bHeightData.size())
                return false;
            
            const size_t sampleCount = std::min<size_t>(20, aHeightData.size());
            for (size_t i = 0; i < sampleCount; ++i)
            {
                size_t index = (i * aHeightData.size()) / sampleCount;
                if (!MathLib::Equal(aHeightData[index], bHeightData[index]))
                    return false;
            }
            
            return true;
        }
        default:
            return false;
    }
}

size_t GenerateHash(const CollisionGeometryCreateOptions& options)
{
    size_t hash = static_cast<size_t>(options.m_GeometryType);

    switch (options.m_GeometryType) {
    case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX: {
        const auto& halfExtents = options.m_BoxParams.m_HalfExtents;
        hash = hash * 31 + static_cast<size_t>(halfExtents[0] * 1000);
        hash = hash * 31 + static_cast<size_t>(halfExtents[1] * 1000);
        hash = hash * 31 + static_cast<size_t>(halfExtents[2] * 1000);
        break;
    }
    case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE: {
        hash = hash * 31 + static_cast<size_t>(options.m_SphereParams.m_Radius * 1000);
        break;
    }
    case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE: {
        hash = hash * 31 + static_cast<size_t>(options.m_CapsuleParams.m_Radius * 1000);
        hash = hash * 31 + static_cast<size_t>(options.m_CapsuleParams.m_HalfHeight * 1000);
        break;
    }
    case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE: {
        const auto& normal = options.m_PlaneParams.m_Normal;
        hash = hash * 31 + static_cast<size_t>(normal[0] * 1000);
        hash = hash * 31 + static_cast<size_t>(normal[1] * 1000);
        hash = hash * 31 + static_cast<size_t>(normal[2] * 1000);
        hash = hash * 31 + static_cast<size_t>(options.m_PlaneParams.m_Distance * 1000);
        break;
    }
    case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
    case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH: {
        const auto& vertices = (options.m_GeometryType == CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH)
            ? options.m_TriangleMeshParams.m_Vertices
            : options.m_ConvexMeshParams.m_Vertices;

        const auto& indices = (options.m_GeometryType == CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH)
            ? options.m_TriangleMeshParams.m_Indices
            : options.m_ConvexMeshParams.m_Indices;

        hash = hash * 31 + vertices.size();
        hash = hash * 31 + indices.size();

        if (!vertices.empty()) {
            const auto& firstVertex = vertices.front();
            hash = hash * 31 + static_cast<size_t>(firstVertex[0] * 100);
            hash = hash * 31 + static_cast<size_t>(firstVertex[1] * 100);
            hash = hash * 31 + static_cast<size_t>(firstVertex[2] * 100);

            if (vertices.size() > 1) {
                const auto& lastVertex = vertices.back();
                hash = hash * 31 + static_cast<size_t>(lastVertex[0] * 100);
                hash = hash * 31 + static_cast<size_t>(lastVertex[1] * 100);
                hash = hash * 31 + static_cast<size_t>(lastVertex[2] * 100);
            }
        }
        break;
    }
    case CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD: {
        const auto& heightData = options.m_HeightFieldParams.m_HeightData;
        hash = hash * 31 + options.m_HeightFieldParams.m_Rows;
        hash = hash * 31 + options.m_HeightFieldParams.m_Columns;
        hash = hash * 31 + static_cast<size_t>(options.m_HeightFieldParams.m_RowScale * 1000);
        hash = hash * 31 + static_cast<size_t>(options.m_HeightFieldParams.m_ColumnScale * 1000);
        hash = hash * 31 + static_cast<size_t>(options.m_HeightFieldParams.m_HeightScale * 1000);

        if (!heightData.empty()) {
            hash = hash * 31 + static_cast<size_t>(heightData.front() * 1000);
            if (heightData.size() > 1) {
                hash = hash * 31 + static_cast<size_t>(heightData.back() * 1000);
            }
        }
        break;
    }
    default:
        break;
    }

    const auto& scale = options.m_Scale;
    hash = hash * 31 + static_cast<size_t>(scale[0] * 1000);
    hash = hash * 31 + static_cast<size_t>(scale[1] * 1000);
    hash = hash * 31 + static_cast<size_t>(scale[2] * 1000);

    return hash;
}