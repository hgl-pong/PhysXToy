#pragma once
#include "Physics/PhysicsCommon.h"
#include "PhysicsRenderObject.h"
namespace Magnum
{
    class Plane
    {
    public:
        Plane() = default;
        Plane(const MathLib::HVector3& normal, MathLib::HReal distance)
            : m_Normal(normal), m_Distance(distance)
        {
        }

        void Set(const MathLib::HVector3& normal, MathLib::HReal distance)
        {
            m_Normal = normal;
            m_Distance = distance;
        }

        const MathLib::HVector3& GetNormal() const
        {
            return m_Normal;
        }

        MathLib::HReal Distance(const MathLib::HVector3& point) const
        {
            return m_Normal.dot(point) + m_Distance;
        }

    private:
        MathLib::HVector3 m_Normal;
        MathLib::HReal m_Distance;
    };

    class FrustumObject
    {
    public:
        void UpdateFrustum(const MathLib::HMatrix4& vpMatrix)
        {
            // 右侧平面
            m_Planes[FRUSTUM_PLANE_RIGHT].Set(
                MathLib::HVector3(vpMatrix(0, 3) - vpMatrix(0, 0),
                    vpMatrix(1, 3) - vpMatrix(1, 0),
                    vpMatrix(2, 3) - vpMatrix(2, 0)),
                vpMatrix(3, 3) - vpMatrix(3, 0));

            // 左侧平面
            m_Planes[FRUSTUM_PLANE_LEFT].Set(
                MathLib::HVector3(vpMatrix(0, 3) + vpMatrix(0, 0),
                    vpMatrix(1, 3) + vpMatrix(1, 0),
                    vpMatrix(2, 3) + vpMatrix(2, 0)),
                vpMatrix(3, 3) + vpMatrix(3, 0));

            // 顶部平面
            m_Planes[FRUSTUM_PLANE_TOP].Set(
                MathLib::HVector3(vpMatrix(0, 3) - vpMatrix(0, 1),
                    vpMatrix(1, 3) - vpMatrix(1, 1),
                    vpMatrix(2, 3) - vpMatrix(2, 1)),
                vpMatrix(3, 3) - vpMatrix(3, 1));

            // 底部平面
            m_Planes[FRUSTUM_PLANE_BOTTOM].Set(
                MathLib::HVector3(vpMatrix(0, 3) + vpMatrix(0, 1),
                    vpMatrix(1, 3) + vpMatrix(1, 1),
                    vpMatrix(2, 3) + vpMatrix(2, 1)),
                vpMatrix(3, 3) + vpMatrix(3, 1));

            // 远平面
            m_Planes[FRUSTUM_PLANE_FAR].Set(
                MathLib::HVector3(vpMatrix(0, 3) - vpMatrix(0, 2),
                    vpMatrix(1, 3) - vpMatrix(1, 2),
                    vpMatrix(2, 3) - vpMatrix(2, 2)),
                vpMatrix(3, 3) - vpMatrix(3, 2));

            // 近平面
            m_Planes[FRUSTUM_PLANE_NEAR].Set(
                MathLib::HVector3(vpMatrix(0, 3) + vpMatrix(0, 2),
                    vpMatrix(1, 3) + vpMatrix(1, 2),
                    vpMatrix(2, 3) + vpMatrix(2, 2)),
                vpMatrix(3, 3) + vpMatrix(3, 2));

            // 归一化平面
            for (int i = 0; i < FRUSTUM_PLANE_COUNT; ++i)
            {
                float length = m_Planes[i].GetNormal().norm();
                m_Planes[i].Set(m_Planes[i].GetNormal() / length, m_Planes[i].Distance(MathLib::HVector3(0, 0, 0)) / length);
            }
        }

        bool IsPointInFrustum(const MathLib::HVector3& point) const
        {
            for (int i = 0; i < FRUSTUM_PLANE_COUNT; i++)
            {
                if (m_Planes[i].Distance(point) < 0)
                {
                    return false;
                }
            }
            return true;
        }

        bool IsSphereInFrustum(const MathLib::HVector3& center, float radius) const
        {
            for (int i = 0; i < FRUSTUM_PLANE_COUNT; i++)
            {
                if (m_Planes[i].Distance(center) < -radius)
                {
                    return false;
                }
            }
            return true;
        }

        bool IsAABBInFrustum(const MathLib::HAABBox3D& aabb) const
        {
            for (int i = 0; i < FRUSTUM_PLANE_COUNT; i++)
            {
                MathLib::HVector3 positiveVertex = aabb.corner(MathLib::HAABBox3D::CornerType::BottomLeft);
                if (m_Planes[i].GetNormal().x() >= 0)
                    positiveVertex.x() = aabb.corner(MathLib::HAABBox3D::CornerType::TopRight).x();
                if (m_Planes[i].GetNormal().y() >= 0)
                    positiveVertex.y() = aabb.corner(MathLib::HAABBox3D::CornerType::TopRight).y();
                if (m_Planes[i].GetNormal().z() >= 0)
                    positiveVertex.z() = aabb.corner(MathLib::HAABBox3D::CornerType::TopRight).z();

                if (m_Planes[i].Distance(positiveVertex) < 0)
                {
                    return false;
                }
            }
            return true;
        }

    private:
        enum FrustumPlane
        {
            FRUSTUM_PLANE_NEAR = 0,
            FRUSTUM_PLANE_FAR,
            FRUSTUM_PLANE_LEFT,
            FRUSTUM_PLANE_RIGHT,
            FRUSTUM_PLANE_TOP,
            FRUSTUM_PLANE_BOTTOM,
            FRUSTUM_PLANE_COUNT
        };

        Plane m_Planes[FRUSTUM_PLANE_COUNT];
    };

    class FrustumCullingManager
    {
    public:
        void UpdateFrustum(const MathLib::HMatrix4& viewProjMatrix)
        {
            m_Frustum.UpdateFrustum(viewProjMatrix);
        }

        void AddObject(std::shared_ptr<PhysicsRenderObject> object)
        {
            m_Objects.push_back(object);
        }

        void RemoveObject(std::shared_ptr<Magnum::PhysicsRenderObject> object)
        {
            auto it = std::find(m_Objects.begin(), m_Objects.end(), object);
            if (it != m_Objects.end())
            {
                m_Objects.erase(it);
            }
        }

        void CullObjects(std::vector<std::shared_ptr<PhysicsRenderObject>>& outObjects)
        {
            outObjects.clear();
            for (auto& object : m_Objects)
            {
                if (m_Frustum.IsAABBInFrustum(object->GetWorldBoundingBox()))
                {
                    outObjects.push_back(object);
                }
            }
        }

    private:
        FrustumObject m_Frustum;
        std::vector<std::shared_ptr<PhysicsRenderObject>> m_Objects;
    };
};