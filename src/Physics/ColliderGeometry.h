#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
	class PxShape;
}

class BoxColliderGeometry : public IColliderGeometry
{
public:
	BoxColliderGeometry(const MathLib::HVector3 &halfExtents) : m_HalfExtents(halfExtents)
	{
		m_BoundingBox = MathLib::HAABBox3D(-halfExtents, halfExtents);
	}
	void Release() override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX; }
	void SetScale(const MathLib::HVector3 &scale) override
	{
		m_Scale = scale;
		const auto halfExtents = MathLib::HadamardProduct<3>(m_HalfExtents, scale);
		m_BoundingBox = MathLib::HAABBox3D(-halfExtents, halfExtents);
	}
	MathLib::HVector3 GetHalfSize() const { return m_HalfExtents; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		options.m_BoxParams.m_HalfExtents = m_HalfExtents;
		options.m_Scale = m_Scale;
	}
	MathLib::HAABBox3D GetBoundingBox() const override { return m_BoundingBox; }

private:
	MathLib::HVector3 m_HalfExtents;
	MathLib::HVector3 m_Scale;
	MathLib::HAABBox3D m_BoundingBox;
};

class SphereColliderGeometry : public IColliderGeometry
{
public:
	SphereColliderGeometry(MathLib::HReal radius) : m_Radius(radius)
	{
		const MathLib::HVector3 extend(radius, radius, radius);
		m_BoundingBox = MathLib::HAABBox3D(-extend, extend);
	}
	void Release() override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE; }
	void SetScale(const MathLib::HVector3 &scale) override
	{
		m_Scale = scale;
		const auto halfExtents = m_Scale * m_Radius;
		m_BoundingBox = MathLib::HAABBox3D(-halfExtents, halfExtents);
	}
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = m_Radius;
		options.m_Scale = m_Scale;
	}
	MathLib::HAABBox3D GetBoundingBox() const override { return m_BoundingBox; }

private:
	MathLib::HReal m_Radius;
	MathLib::HVector3 m_Scale;
	MathLib::HAABBox3D m_BoundingBox;
};

class PlaneColliderGeometry : public IColliderGeometry
{
public:
	PlaneColliderGeometry(const MathLib::HVector3 &normal, MathLib::HReal distance) : m_Normal(normal), m_Distance(distance)
	{
	}
	void Release() override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE; }
	void SetScale(const MathLib::HVector3 &scale) override
	{
		m_Scale = scale;
	}
	MathLib::HVector3 GetNormal() const { return m_Normal; }
	MathLib::HReal GetDistance() const { return m_Distance; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE;
		options.m_PlaneParams.m_Normal = m_Normal;
		options.m_PlaneParams.m_Distance = m_Distance;
		options.m_Scale = m_Scale;
	}
	MathLib::HAABBox3D GetBoundingBox() const override { return MathLib::HAABBox3D(); }

private:
	MathLib::HVector3 m_Normal;
	MathLib::HReal m_Distance;
	MathLib::HVector3 m_Scale;
};

class CapsuleColliderGeometry : public IColliderGeometry
{
public:
	CapsuleColliderGeometry(MathLib::HReal radius, MathLib::HReal halfHeight) : m_Radius(radius), m_HalfHeight(halfHeight)
	{
		m_BoundingBox = MathLib::HAABBox3D(-MathLib::HVector3(radius, halfHeight, radius), MathLib::HVector3(radius, halfHeight, radius));
	}
	void Release() override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE; }
	void SetScale(const MathLib::HVector3 &scale) override
	{
		m_Scale = scale;
		const auto halfExtents = MathLib::HadamardProduct<3>(MathLib::HVector3(m_HalfHeight + m_Radius,m_Radius, m_Radius), scale);
		m_BoundingBox = MathLib::HAABBox3D(-halfExtents, halfExtents);
	}
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HReal GetHalfHeight() const { return m_HalfHeight; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
		options.m_CapsuleParams.m_Radius = m_Radius;
		options.m_CapsuleParams.m_HalfHeight = m_HalfHeight;
		options.m_Scale = m_Scale;
	}
	MathLib::HAABBox3D GetBoundingBox() const override { return m_BoundingBox; }

private:
	MathLib::HReal m_Radius;
	MathLib::HReal m_HalfHeight;
	MathLib::HVector3 m_Scale;
	MathLib::HAABBox3D m_BoundingBox;
};

class TriangleMeshColliderGeometry : public IColliderGeometry
{
public:
	TriangleMeshColliderGeometry(const std::vector<MathLib::HVector3> &vertices, const std::vector<uint32_t> &indices) : m_Vertices(vertices), m_Indices(indices)
	{
		for (const auto &v : vertices)
			m_BoundingBox.extend(v);
	}
	void Release() override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH; }
	void SetScale(const MathLib::HVector3 &scale) override
	{
		m_Scale = scale;
		m_BoundingBox.setEmpty();
		for (const auto &v : m_Vertices)
			m_BoundingBox.extend(MathLib::HadamardProduct<3>(v, scale));
	}
	const std::vector<MathLib::HVector3> &GetVertices() const { return m_Vertices; }
	const std::vector<uint32_t> &GetIndices() const { return m_Indices; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH;
		options.m_TriangleMeshParams.m_Vertices = m_Vertices;
		options.m_TriangleMeshParams.m_Indices = m_Indices;
		options.m_Scale = m_Scale;
	}
	MathLib::HAABBox3D GetBoundingBox() const override { return m_BoundingBox; }

private:
	std::vector<MathLib::HVector3> m_Vertices;
	std::vector<uint32_t> m_Indices;
	MathLib::HVector3 m_Scale;
	MathLib::HAABBox3D m_BoundingBox;
};

class ConvexMeshColliderGeometry : public IColliderGeometry
{
public:
	ConvexMeshColliderGeometry(const std::vector<MathLib::HVector3> &vertices, const std::vector<uint32_t> &indices) : m_Vertices(vertices), m_Indices(indices)
	{
		for (const auto &v : vertices)
			m_BoundingBox.extend(v);
	}
	void Release() override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH; }
	void SetScale(const MathLib::HVector3 &scale) override
	{
		m_Scale = scale;
		m_BoundingBox.setEmpty();
		for (const auto &v : m_Vertices)
			m_BoundingBox.extend(MathLib::HadamardProduct<3>(v, scale));
	}
	const std::vector<MathLib::HVector3> &GetVertices() const { return m_Vertices; }
	const std::vector<uint32_t> &GetIndices() const { return m_Indices; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
		options.m_ConvexMeshParams.m_Vertices = m_Vertices;
		options.m_ConvexMeshParams.m_Indices = m_Indices;
		options.m_Scale = m_Scale;
	}
	MathLib::HAABBox3D GetBoundingBox() const override
	{
		return m_BoundingBox;
	}

private:
	std::vector<MathLib::HVector3> m_Vertices;
	std::vector<uint32_t> m_Indices;
	MathLib::HVector3 m_Scale;
	MathLib::HAABBox3D m_BoundingBox;
};