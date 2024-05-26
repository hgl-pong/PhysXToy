#pragma once
#include <Eigen/Dense>
namespace MathLib
{
	typedef float HReal;
	typedef Eigen::Matrix<HReal, 3, 1> HVector3;
	typedef Eigen::Matrix<HReal, 4, 1> HVector4;

	typedef Eigen::Transform<HReal, 3, Eigen::Affine> HTransform3;
}

struct PhysicsEngineOptions
{
	uint32_t m_iNumThreads;
	bool m_bEnablePVD;
};

enum class PhysicsSceneFilterShaderType
{
	eDEFAULT
};

struct PhysicsSceneCreateOptions
{
	MathLib::HVector3 m_Gravity;
	PhysicsSceneFilterShaderType m_FilterShaderType;
};

struct PhysicsMaterialCreateOptions
{
	MathLib::HReal m_StaticFriction;
	MathLib::HReal m_DynamicFriction;
	MathLib::HReal m_Restitution;
	MathLib::HReal m_Density;
};

enum class CollierGeometryType
{
	COLLIER_GEOMETRY_TYPE_SPHERE,
	COLLIER_GEOMETRY_TYPE_BOX,
	COLLIER_GEOMETRY_TYPE_CAPSULE,
	COLLIER_GEOMETRY_TYPE_CYLINDER,
	COLLIER_GEOMETRY_TYPE_CONE,
	COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH,
	COLLIER_GEOMETRY_TYPE_CONVEX_MESH,
	COLLIER_GEOMETRY_TYPE_COUNT
};

struct CollisionGeometryCreateOptions
{
	CollierGeometryType m_GeometryType;
	MathLib::HTransform3 m_LocalTransform;
	struct SphereParams
	{
		MathLib::HReal m_Radius;
	} m_SphereParams;
	struct BoxParams
	{
		MathLib::HVector3 m_HalfExtents;
	} m_BoxParams;
	struct CapsuleParams
	{
		MathLib::HReal m_Radius;
		MathLib::HReal m_Height;
	} m_CapsuleParams;
	struct CylinderParams
	{
		MathLib::HReal m_Radius;
		MathLib::HReal m_Height;
	} m_CylinderParams;
	struct ConeParams
	{
		MathLib::HReal m_Radius;
		MathLib::HReal m_Height;
	} m_ConeParams;
	struct TriangleMeshParams
	{
		MathLib::HVector3 *m_Vertices;
		uint32_t m_iNumVertices;
		uint32_t *m_Indices;
		uint32_t m_iNumIndices;
	} m_TriangleMeshParams;
	struct ConvexMeshParams
	{
		MathLib::HVector3 *m_Vertices;
		uint32_t m_iNumVertices;
	} m_ConvexMeshParams;
};

enum class PhysicsObjectType
{
	PHYSICS_OBJECT_TYPE_RIGID_STATIC,
	PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC,
	PHYSICS_OBJECT_TYPE_RIGID_KINEMATIC,
	PHYSICS_OBJECT_TYPE_COUNT
};