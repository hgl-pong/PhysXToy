#pragma once
#include "Physics/PhysicsCommon.h"
#include "Base/ObjectCache.h"
#include "Utility/PhysicsUtils.h"

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
	void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) override { m_Material = material; }
	PhysicsPtr<IPhysicsMaterial> GetMaterial() const override { return m_Material; }
	bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) override
	{
		MathLib::HAABBox3D worldBox = GetBoundingBox();
		worldBox.transform(worldTransform);
		
		MathLib::HVector3 invDir(
			ray.GetDirection()[0] != 0 ? 1.0f / ray.GetDirection()[0] : 0,
			ray.GetDirection()[1] != 0 ? 1.0f / ray.GetDirection()[1] : 0,
			ray.GetDirection()[2] != 0 ? 1.0f / ray.GetDirection()[2] : 0
		);
		
		MathLib::HVector3 t0 = MathLib::HadamardProduct<3>((worldBox.min() - ray.GetOrigin()), invDir);
		MathLib::HVector3 t1 = MathLib::HadamardProduct<3>((worldBox.max() - ray.GetOrigin()), invDir);
		
		MathLib::HVector3 tmin = MathLib::Min(t0, t1);
		MathLib::HVector3 tmax = MathLib::Max(t0, t1);
		
		MathLib::HReal distMin = MathLib::Max(tmin[0], MathLib::Max(tmin[1], tmin[2]));
		MathLib::HReal distMax = MathLib::Min(tmax[0], MathLib::Min(tmax[1], tmax[2]));
		
		if (distMax < 0 || distMin > distMax)
			return false;
			
		distance = distMin >= 0 ? distMin : distMax;
		
		MathLib::HVector3 hitPoint = ray.GetOrigin() + ray.GetDirection() * distance;
		MathLib::HVector3 center = worldTransform.translation();
		MathLib::HVector3 dir = (hitPoint - center).normalized();
		
		MathLib::HVector3 localDir = worldTransform.rotation().transpose() * dir;
		MathLib::HVector3 absDir = MathLib::HVector3(
			MathLib::Abs(localDir[0]), 
			MathLib::Abs(localDir[1]), 
			MathLib::Abs(localDir[2])
		);

		if (absDir[0] > absDir[1] && absDir[0] > absDir[2])
			normal = worldTransform.rotation() * MathLib::HVector3(localDir[0] > 0 ? 1 : -1, 0, 0);
		else if (absDir[1] > absDir[0] && absDir[1] > absDir[2])
			normal = worldTransform.rotation() * MathLib::HVector3(0, localDir[1] > 0 ? 1 : -1, 0);
		else
			normal = worldTransform.rotation() * MathLib::HVector3(0, 0, localDir[2] > 0 ? 1 : -1);
		
		return true;
	}
	
	MathLib::HVector3 GetHalfSize() const { return m_HalfExtents; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options) override
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
	PhysicsPtr<IPhysicsMaterial> m_Material;
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
	void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) override { m_Material = material; }
	PhysicsPtr<IPhysicsMaterial> GetMaterial() const override { return m_Material; }
	bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) override
	{
		MathLib::HVector3 center = worldTransform.translation();
		MathLib::HReal scaledRadius = m_Radius * (m_Scale[0] + m_Scale[1] + m_Scale[2]) / 3.0f;
		
		MathLib::HVector3 oc = ray.GetOrigin() - center;
		MathLib::HReal a = ray.GetDirection().norm();
		MathLib::HReal b = 2.0f * oc.dot(ray.GetDirection());
		MathLib::HReal c = oc.norm() - scaledRadius * scaledRadius;
		MathLib::HReal discriminant = b * b - 4 * a * c;
		
		if (discriminant < 0)
			return false;
			
		MathLib::HReal t = (-b - sqrt(discriminant)) / (2.0f * a);
		if (t < 0) {
			t = (-b + sqrt(discriminant)) / (2.0f * a);
			if (t < 0)
				return false;
		}
		
		distance = t;
		
		MathLib::HVector3 hitPoint = ray.GetOrigin() + ray.GetDirection() * distance;
		normal = (hitPoint - center).normalized();
		
		return true;
	}
	
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options) override
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
	PhysicsPtr<IPhysicsMaterial> m_Material;
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
	void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) override { m_Material = material; }
	PhysicsPtr<IPhysicsMaterial> GetMaterial() const override { return m_Material; }
	bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) override
	{
		MathLib::HVector3 worldNormal = worldTransform.rotation() * m_Normal;
		worldNormal.normalize();
		
		MathLib::HReal denom = worldNormal.dot(ray.GetDirection());
		if (MathLib::Abs(denom) < 1e-6) 
			return false; 
			
		MathLib::HVector3 planePos = worldTransform.translation() + worldNormal * m_Distance;
		MathLib::HReal t = (planePos - ray.GetOrigin()).dot(worldNormal) / denom;
		
		if (t < 0)
			return false; 
			
		distance = t;
		normal = denom < 0 ? worldNormal : -worldNormal;
		
		return true;
	}
	
	MathLib::HVector3 GetNormal() const { return m_Normal; }
	MathLib::HReal GetDistance() const { return m_Distance; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options) override
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
	PhysicsPtr<IPhysicsMaterial> m_Material;
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
	void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) override { m_Material = material; }
	PhysicsPtr<IPhysicsMaterial> GetMaterial() const override { return m_Material; }
	bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) override
	{
		MathLib::HAABBox3D worldBox = GetBoundingBox();
		worldBox.transform(worldTransform);
		
		MathLib::HVector3 invDir(
			ray.GetDirection()[0] != 0 ? 1.0f / ray.GetDirection()[0] : 0,
			ray.GetDirection()[1] != 0 ? 1.0f / ray.GetDirection()[1] : 0,
			ray.GetDirection()[2] != 0 ? 1.0f / ray.GetDirection()[2] : 0
		);
		
		MathLib::HVector3 t0 = MathLib::HadamardProduct<3>((worldBox.min() - ray.GetOrigin()), invDir);
		MathLib::HVector3 t1 = MathLib::HadamardProduct<3>((worldBox.max() - ray.GetOrigin()), invDir);
		
		MathLib::HVector3 tmin = MathLib::Min(t0, t1);
		MathLib::HVector3 tmax = MathLib::Max(t0, t1);
		
		MathLib::HReal distMin = MathLib::Max(tmin[0], MathLib::Max(tmin[1], tmin[2]));
		MathLib::HReal distMax = MathLib::Min(tmax[0], MathLib::Min(tmax[1], tmax[2]));
		
		if (distMax < 0 || distMin > distMax)
			return false;
		
		distance = distMin >= 0 ? distMin : distMax;
		
		MathLib::HVector3 hitPoint = ray.GetOrigin() + ray.GetDirection() * distance;
		normal = (hitPoint - worldTransform.translation()).normalized();
		
		return true;
	}
	
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HReal GetHalfHeight() const { return m_HalfHeight; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options) override
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
	PhysicsPtr<IPhysicsMaterial> m_Material;
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
	void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) override { m_Material = material; }
	PhysicsPtr<IPhysicsMaterial> GetMaterial() const override { return m_Material; }
	bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) override
	{
		MathLib::HAABBox3D worldBox = GetBoundingBox();
		worldBox.transform(worldTransform);
		
		MathLib::HVector3 invDir(
			ray.GetDirection()[0] != 0 ? 1.0f / ray.GetDirection()[0] : 0,
			ray.GetDirection()[1] != 0 ? 1.0f / ray.GetDirection()[1] : 0,
			ray.GetDirection()[2] != 0 ? 1.0f / ray.GetDirection()[2] : 0
		);
		
		MathLib::HVector3 t0 = MathLib::HadamardProduct<3>((worldBox.min() - ray.GetOrigin()), invDir);
		MathLib::HVector3 t1 = MathLib::HadamardProduct<3>((worldBox.max() - ray.GetOrigin()), invDir);
		
		MathLib::HVector3 tmin = MathLib::Min(t0, t1);
		MathLib::HVector3 tmax = MathLib::Max(t0, t1);
		
		MathLib::HReal distMin = MathLib::Max(tmin[0], MathLib::Max(tmin[1], tmin[2]));
		MathLib::HReal distMax = MathLib::Min(tmax[0], MathLib::Min(tmax[1], tmax[2]));
		
		if (distMax < 0 || distMin > distMax)
			return false;
			
		distance = distMin >= 0 ? distMin : distMax;
		normal = MathLib::HVector3(0, 1, 0); 
		
		return true;
	}
	
	const std::vector<MathLib::HVector3> &GetVertices() const { return m_Vertices; }
	const std::vector<uint32_t> &GetIndices() const { return m_Indices; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options) override
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
	PhysicsPtr<IPhysicsMaterial> m_Material;
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
	void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) override { m_Material = material; }
	PhysicsPtr<IPhysicsMaterial> GetMaterial() const override { return m_Material; }
	bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) override
	{
		MathLib::HAABBox3D worldBox = GetBoundingBox();
		worldBox.transform(worldTransform);
		
		MathLib::HVector3 invDir(
			ray.GetDirection()[0] != 0 ? 1.0f / ray.GetDirection()[0] : 0,
			ray.GetDirection()[1] != 0 ? 1.0f / ray.GetDirection()[1] : 0,
			ray.GetDirection()[2] != 0 ? 1.0f / ray.GetDirection()[2] : 0
		);
		
		MathLib::HVector3 t0 = MathLib::HadamardProduct<3>((worldBox.min() - ray.GetOrigin()), invDir);
		MathLib::HVector3 t1 = MathLib::HadamardProduct<3>((worldBox.max() - ray.GetOrigin()), invDir);
		
		MathLib::HVector3 tmin = MathLib::Min(t0, t1);
		MathLib::HVector3 tmax = MathLib::Max(t0, t1);
		
		MathLib::HReal distMin = MathLib::Max(tmin[0], MathLib::Max(tmin[1], tmin[2]));
		MathLib::HReal distMax = MathLib::Min(tmax[0], MathLib::Min(tmax[1], tmax[2]));
		
		if (distMax < 0 || distMin > distMax)
			return false;
			
		distance = distMin >= 0 ? distMin : distMax;
		
		MathLib::HVector3 hitPoint = ray.GetOrigin() + ray.GetDirection() * distance;
		normal = (hitPoint - worldTransform.translation()).normalized();
		
		return true;
	}
	
	const std::vector<MathLib::HVector3> &GetVertices() const { return m_Vertices; }
	const std::vector<uint32_t> &GetIndices() const { return m_Indices; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	void GetParams(CollisionGeometryCreateOptions &options) override
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
		options.m_ConvexMeshParams.m_Vertices = m_Vertices;
		options.m_ConvexMeshParams.m_Indices = m_Indices;
		options.m_Scale = m_Scale;
	}
	MathLib::HAABBox3D GetBoundingBox() const override { return m_BoundingBox; }

private:
	std::vector<MathLib::HVector3> m_Vertices;
	std::vector<uint32_t> m_Indices;
	MathLib::HVector3 m_Scale;
	MathLib::HAABBox3D m_BoundingBox;
	PhysicsPtr<IPhysicsMaterial> m_Material;
};

class HeightFieldColliderGeometry : public IColliderGeometry
{
public:
	HeightFieldColliderGeometry(const std::vector<MathLib::HReal>& heightData, int rows, int columns, 
		MathLib::HReal rowScale = 1.0f, MathLib::HReal columnScale = 1.0f, MathLib::HReal heightScale = 1.0f)
		: m_HeightData(heightData), m_Rows(rows), m_Columns(columns), 
		  m_RowScale(rowScale), m_ColumnScale(columnScale), m_HeightScale(heightScale)
	{
		float minHeight = std::numeric_limits<float>::max();
		float maxHeight = -std::numeric_limits<float>::max();
		
		for (auto height : m_HeightData)
		{
			minHeight = MathLib::Min(minHeight, height);
			maxHeight = MathLib::Max(maxHeight, height);
		}
		
		MathLib::HVector3 min(-m_ColumnScale * m_Columns * 0.5f, minHeight * m_HeightScale, -m_RowScale * m_Rows * 0.5f);
		MathLib::HVector3 max(m_ColumnScale * m_Columns * 0.5f, maxHeight * m_HeightScale, m_RowScale * m_Rows * 0.5f);
		m_BoundingBox = MathLib::HAABBox3D(min, max);
	}
	
	void Release() override {}
	
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD; }
	
	void SetScale(const MathLib::HVector3 &scale) override
	{
		m_Scale = scale;
		
		float minHeight = std::numeric_limits<float>::max();
		float maxHeight = -std::numeric_limits<float>::max();
		
		for (auto height : m_HeightData)
		{
			minHeight = MathLib::Min(minHeight, height);
			maxHeight = MathLib::Max(maxHeight, height);
		}
		
		MathLib::HVector3 min(-m_ColumnScale * m_Columns * 0.5f * scale[0], 
							  minHeight * m_HeightScale * scale[1], 
							  -m_RowScale * m_Rows * 0.5f * scale[2]);
		MathLib::HVector3 max(m_ColumnScale * m_Columns * 0.5f * scale[0], 
							 maxHeight * m_HeightScale * scale[1], 
							 m_RowScale * m_Rows * 0.5f * scale[2]);
		m_BoundingBox = MathLib::HAABBox3D(min, max);
	}
	
	void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) override { m_Material = material; }
	
	PhysicsPtr<IPhysicsMaterial> GetMaterial() const override { return m_Material; }
	
	bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) override
	{
		MathLib::HAABBox3D worldBox = GetBoundingBox();
		worldBox.transform(worldTransform);
		
		MathLib::HVector3 invDir(
			ray.GetDirection()[0] != 0 ? 1.0f / ray.GetDirection()[0] : 0,
			ray.GetDirection()[1] != 0 ? 1.0f / ray.GetDirection()[1] : 0,
			ray.GetDirection()[2] != 0 ? 1.0f / ray.GetDirection()[2] : 0
		);
		
		MathLib::HVector3 t0 = MathLib::HadamardProduct<3>((worldBox.min() - ray.GetOrigin()), invDir);
		MathLib::HVector3 t1 = MathLib::HadamardProduct<3>((worldBox.max() - ray.GetOrigin()), invDir);
		
		MathLib::HVector3 tmin = MathLib::Min(t0, t1);
		MathLib::HVector3 tmax = MathLib::Max(t0, t1);
		
		MathLib::HReal distMin = MathLib::Max(tmin[0], MathLib::Max(tmin[1], tmin[2]));
		MathLib::HReal distMax = MathLib::Min(tmax[0], MathLib::Min(tmax[1], tmax[2]));
		
		if (distMax < 0 || distMin > distMax)
			return false;
			
		distance = distMin >= 0 ? distMin : distMax;
		normal = MathLib::HVector3(0, 1, 0); 
		
		return true;
	}
	
	const std::vector<MathLib::HReal>& GetHeightData() const { return m_HeightData; }
	int GetRows() const { return m_Rows; }
	int GetColumns() const { return m_Columns; }
	MathLib::HReal GetRowScale() const { return m_RowScale; }
	MathLib::HReal GetColumnScale() const { return m_ColumnScale; }
	MathLib::HReal GetHeightScale() const { return m_HeightScale; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
	
	void GetParams(CollisionGeometryCreateOptions &options) override
	{
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD;
		options.m_HeightFieldParams.m_HeightData = m_HeightData;
		options.m_HeightFieldParams.m_Rows = m_Rows;
		options.m_HeightFieldParams.m_Columns = m_Columns;
		options.m_HeightFieldParams.m_RowScale = m_RowScale;
		options.m_HeightFieldParams.m_ColumnScale = m_ColumnScale;
		options.m_HeightFieldParams.m_HeightScale = m_HeightScale;
		options.m_Scale = m_Scale;
	}
	
	MathLib::HAABBox3D GetBoundingBox() const override { return m_BoundingBox; }

private:
	std::vector<MathLib::HReal> m_HeightData;
	int m_Rows;
	int m_Columns;
	MathLib::HReal m_RowScale;
	MathLib::HReal m_ColumnScale;
	MathLib::HReal m_HeightScale;
	MathLib::HVector3 m_Scale;
	MathLib::HAABBox3D m_BoundingBox;
	PhysicsPtr<IPhysicsMaterial> m_Material;
};


extern bool AreGeometriesEqual(const CollisionGeometryCreateOptions& a, const CollisionGeometryCreateOptions& b);
extern size_t GenerateHash(const CollisionGeometryCreateOptions& options);

namespace PhysicsBase
{
    template<>
    struct Creator<CollisionGeometryCreateOptions, IColliderGeometry>
    {
        PhysicsPtr<IColliderGeometry> Create(const CollisionGeometryCreateOptions& options)
        {
            return PhysicsCacheUtils::CreateColliderGeometry(options);
        }
    };
}

namespace std
{
    template <>
    struct hash<CollisionGeometryCreateOptions> {
        size_t operator()(const CollisionGeometryCreateOptions& options) const
        {
            return GenerateHash(options);
        }
    };

    template<>
    struct equal_to<CollisionGeometryCreateOptions>
    {
        bool operator()(const CollisionGeometryCreateOptions& a, const CollisionGeometryCreateOptions& b) const
        {
            return AreGeometriesEqual(a, b);
        }
    };
}

using ColliderGeometryCache = ObjectCache<IColliderGeometry, CollisionGeometryCreateOptions>;
