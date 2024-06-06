#pragma once
#include "Physics/PhysicsCommon.h"
#include "PhysicsRenderObject.h"
#include "Camera.h"
namespace Magnum
{
    class Plane
    {
    public:
        Plane() = default;
        Plane(const MathLib::HVector3 &normal, MathLib::HReal distance)
            : m_Normal(normal), m_Distance(distance)
        {
        }

        void Set(const MathLib::HVector3& position, const MathLib::HVector3& normal)
        {
            m_Normal = normal.normalized();
			m_Distance = m_Normal.dot(position);
		}

        const MathLib::HVector3 &GetNormal() const
        {
            return m_Normal;
        }

        MathLib::HReal Distance(const MathLib::HVector3& point) const
        {
            return m_Normal.dot(point) - m_Distance;
        }

    private:
        MathLib::HVector3 m_Normal;
        MathLib::HReal m_Distance =0.f;
    };

    class FrustumObject
    {
    public:
        FrustumObject(MathLib::Camera &camera) : m_Camera(camera)
        {
        }
        void UpdateFrustum()
        {
            const MathLib::HVector3& eye = m_Camera.getEye();
            const MathLib::HVector3& dir = m_Camera.getDir();
            const MathLib::HVector3& right = MathLib::HVector3(0, 1, 0).cross(-dir).normalized();
            const MathLib::HVector3& up = (-dir).cross(right);
            const MathLib::HReal aspectRatio = m_Camera.getAspectRatio();
            const MathLib::HReal nearClip = m_Camera.getNearClip();
            const MathLib::HReal farClip = m_Camera.getFarClip();
            const MathLib::HReal fov = m_Camera.getFOV();

            const MathLib::HReal halfHSide = std::tan(fov / 2 * 4/3 * MathLib::H_PI / 180.f) * farClip;
            const MathLib::HReal halfVSide = halfHSide / aspectRatio;
            const MathLib::HVector3 frontMultFar = dir * farClip;

            m_Planes[FRUSTUM_PLANE_NEAR].Set(eye + dir * nearClip, dir);
            m_Planes[FRUSTUM_PLANE_FAR].Set(eye + frontMultFar, -dir);
            m_Planes[FRUSTUM_PLANE_RIGHT].Set(eye, -(frontMultFar + right * halfHSide).cross(up));
            m_Planes[FRUSTUM_PLANE_LEFT].Set(eye, -up.cross(frontMultFar-right* halfHSide));
            m_Planes[FRUSTUM_PLANE_TOP].Set(eye, -right.cross(frontMultFar+up* halfVSide));
            m_Planes[FRUSTUM_PLANE_BOTTOM].Set(eye, -(frontMultFar - up * halfVSide).cross(right));
        }

        bool IsPointInFrustum(const MathLib::HVector3 &point) const
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

        bool IsSphereInFrustum(const MathLib::HVector3 &center, float radius) const
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
            Eigen::Vector3f center = aabb.center();
            Eigen::Vector3f extents = aabb.sizes() / 2.0f;

            for (int i = 0; i < FRUSTUM_PLANE_COUNT; ++i)
            {
                const Eigen::Vector3f& normal = m_Planes[i].GetNormal();
                float distance = m_Planes[i].Distance(center);
                float r = extents.x() * std::abs(normal.x()) +
                    extents.y() * std::abs(normal.y()) +
                    extents.z() * std::abs(normal.z());

                if (distance < -r)
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
        MathLib::Camera &m_Camera;
        Plane m_Planes[FRUSTUM_PLANE_COUNT];
    };

    class FrustumCullingManager
    {
    public:
        FrustumCullingManager(MathLib::Camera &camera) : m_Frustum(camera)
        {
        }
        void UpdateFrustum()
        {
            m_Frustum.UpdateFrustum();
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

        void CullObjects(std::vector<std::shared_ptr<PhysicsRenderObject>> &outObjects)
        {
            outObjects.clear();
            for (auto &object : m_Objects)
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