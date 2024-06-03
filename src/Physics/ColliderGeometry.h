#pragma once
#include "Physics/PhysicsCommon.h"
namespace physx
{
	class PxShape;
}

class BoxColliderGeometry : public IColliderGeometry
{
public:
	BoxColliderGeometry(const MathLib::HVector3 &halfExtents) : m_HalfExtents(halfExtents) {}
	void Release()override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	MathLib::HVector3 GetHalfSize() const { return m_HalfExtents; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions& options)
	{
		options.m_GeometryType=CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		options.m_BoxParams.m_HalfExtents = m_HalfExtents;
		options.m_Scale = m_Scale;
	}
private:
	MathLib::HVector3 m_HalfExtents;
	MathLib::HVector3 m_Scale;
};

class SphereColliderGeometry : public IColliderGeometry
{
public:
	SphereColliderGeometry(MathLib::HReal radius) : m_Radius(radius) {}
	void Release()override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions& options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = m_Radius;
		options.m_Scale = m_Scale;
	}
private:
	MathLib::HReal m_Radius;
	MathLib::HVector3 m_Scale;
};

class PlaneColliderGeometry : public IColliderGeometry
{
public:
	PlaneColliderGeometry(const MathLib::HVector3 &normal, MathLib::HReal distance) : m_Normal(normal), m_Distance(distance) {}
	void Release()override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	MathLib::HVector3 GetNormal() const { return m_Normal; }
	MathLib::HReal GetDistance() const { return m_Distance; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions& options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE;
		options.m_PlaneParams.m_Normal = m_Normal;
		options.m_PlaneParams.m_Distance = m_Distance;
		options.m_Scale = m_Scale;
	}
private:
	MathLib::HVector3 m_Normal;
	MathLib::HReal m_Distance;
	MathLib::HVector3 m_Scale;
};

class CapsuleColliderGeometry : public IColliderGeometry
{
public:
	CapsuleColliderGeometry(MathLib::HReal radius, MathLib::HReal halfHeight) : m_Radius(radius), m_HalfHeight(halfHeight) {}
	void Release()override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE; }
	void SetScale(const MathLib::HVector3& scale) override { m_Scale = scale; }
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HReal GetHalfHeight() const { return m_HalfHeight; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions& options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
		options.m_CapsuleParams.m_Radius = m_Radius;
		options.m_CapsuleParams.m_HalfHeight = m_HalfHeight;
		options.m_Scale = m_Scale;
	}
private:
	MathLib::HReal m_Radius;
	MathLib::HReal m_HalfHeight;
	MathLib::HVector3 m_Scale;
};

class TriangleMeshColliderGeometry : public IColliderGeometry
{
public:
	TriangleMeshColliderGeometry(const std::vector<MathLib::HVector3> &vertices, const std::vector<uint32_t> &indices) : m_Vertices(vertices), m_Indices(indices) {}
	void Release()override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	const std::vector<MathLib::HVector3> &GetVertices() const { return m_Vertices; }
	const std::vector<uint32_t> &GetIndices() const { return m_Indices; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions& options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH;
		options.m_TriangleMeshParams.m_Vertices = m_Vertices;
		options.m_TriangleMeshParams.m_Indices = m_Indices;
		options.m_Scale = m_Scale;
	}
private:
	std::vector<MathLib::HVector3> m_Vertices;
	std::vector<uint32_t> m_Indices;
	MathLib::HVector3 m_Scale;
};

class ConvexMeshColliderGeometry : public IColliderGeometry
{
public:
	ConvexMeshColliderGeometry(const std::vector<MathLib::HVector3> &vertices) : m_Vertices(vertices) {}
	void Release()override {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	const std::vector<MathLib::HVector3> &GetVertices() const { return m_Vertices; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions& options)
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
		options.m_ConvexMeshParams.m_Vertices = m_Vertices;
		options.m_Scale = m_Scale;
	}
private:
	std::vector<MathLib::HVector3> m_Vertices;
	MathLib::HVector3 m_Scale;
};