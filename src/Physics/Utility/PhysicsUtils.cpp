#include "PhysicsUtils.h"
#include "../ColliderGeometry.h"
#include "../Base/ObjectCache.h"
#include "../PhysicsCache.h"

namespace PhysicsCacheUtils
{
    PhysicsPtr<IColliderGeometry> CreateColliderGeometry(const CollisionGeometryCreateOptions& options)
    {
        IColliderGeometry* geometry = nullptr;
        switch (options.m_GeometryType)
        {
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
        {
            geometry = new BoxColliderGeometry(options.m_BoxParams.m_HalfExtents);
            break;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
        {
            geometry = new SphereColliderGeometry(options.m_SphereParams.m_Radius);
            break;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
        {
            geometry = new PlaneColliderGeometry(options.m_PlaneParams.m_Normal, options.m_PlaneParams.m_Distance);
            break;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
        {
            geometry = new CapsuleColliderGeometry(options.m_CapsuleParams.m_Radius, options.m_CapsuleParams.m_HalfHeight);
            break;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
        {
            geometry = new TriangleMeshColliderGeometry(options.m_TriangleMeshParams.m_Vertices, options.m_TriangleMeshParams.m_Indices);
            break;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
        {
            geometry = new ConvexMeshColliderGeometry(options.m_ConvexMeshParams.m_Vertices, options.m_ConvexMeshParams.m_Indices);
            break;
        }
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD:
        {
            geometry = new HeightFieldColliderGeometry(
                options.m_HeightFieldParams.m_HeightData,
                options.m_HeightFieldParams.m_Rows,
                options.m_HeightFieldParams.m_Columns,
                options.m_HeightFieldParams.m_RowScale,
                options.m_HeightFieldParams.m_ColumnScale,
                options.m_HeightFieldParams.m_HeightScale
            );
            break;
        }
        default:
            break;
        }
        if (geometry)
        {
            geometry->SetScale(options.m_Scale);
        }
        return make_physics_ptr(geometry);
    }

    void CleanupUnusedGeometries()
    {
        ColliderGeometryCache::GetInstance().CleanupUnused();
    }

    size_t GetGeometryCacheSize()
    {
        return ColliderGeometryCache::GetInstance().Size();
    }

    void ClearGeometryCache()
    {
        ColliderGeometryCache::GetInstance().Clear();
    }
}
