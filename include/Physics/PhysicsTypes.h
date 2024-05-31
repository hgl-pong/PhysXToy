#pragma once
#include <Eigen/Dense>
namespace MathLib
{
#define H_PI 3.14159265358979323846f
	typedef float HReal;
	typedef Eigen::Matrix<HReal, 3, 1> HVector3;
	typedef Eigen::Matrix<HReal, 4, 1> HVector4;

	typedef Eigen::Matrix<HReal, 3, 3> HMatrix3;
	typedef Eigen::Matrix<HReal, 4, 4> HMatrix4;

	typedef Eigen::Quaternion<HReal> HQuaternion;
	typedef Eigen::AngleAxis<HReal> HAngleAxis;

	typedef Eigen::AlignedBox<HReal, 3> HAABBox3D;

	typedef Eigen::Transform<HReal, 3, Eigen::Affine> HTransform3;

	typedef Eigen::Translation<HReal, 3> HTranslation3;
}
#define DEFAULT_CPU_DISPATCHER_NUM_THREADS 2

template <typename T>
struct PhysicsDeleter
{
	void operator()(T* ptr) const {
		if (ptr)
		{
			ptr->Release();
			delete ptr;
			ptr = nullptr;
		}
	}
};

template <typename T>
using PhysicsPtr = std::unique_ptr<T, PhysicsDeleter<T>>;
template <typename T>
PhysicsPtr<T> make_physics_ptr(T* ptr) {
	return PhysicsPtr<T>(ptr);
}

template <typename T>
struct PhysXDeleter
{
	void operator()(T* ptr) const {
		if (ptr)
		{ 
			ptr->release();
			ptr = nullptr;
		}
	}
};

template <typename T>
using PhysXPtr = std::unique_ptr<T, PhysXDeleter<T>>;

template <typename T>
PhysXPtr<T> make_physx_ptr(T* ptr) {
	return PhysXPtr<T>(ptr);
}

struct PhysicsEngineOptions
{
	uint32_t m_iNumThreads= DEFAULT_CPU_DISPATCHER_NUM_THREADS;
	bool m_bEnablePVD=true;
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
	MathLib::HReal m_StaticFriction=0.5;
	MathLib::HReal m_DynamicFriction=0.5;
	MathLib::HReal m_Restitution=0.6;
	MathLib::HReal m_Density=10;
};

enum class CollierGeometryType
{
	COLLIER_GEOMETRY_TYPE_SPHERE,
	COLLIER_GEOMETRY_TYPE_BOX,
	COLLIER_GEOMETRY_TYPE_CAPSULE,
	COLLIER_GEOMETRY_TYPE_PLANE,
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
		MathLib::HReal m_Radius=1;
	} m_SphereParams;
	struct BoxParams
	{
		MathLib::HVector3 m_HalfExtents = {1,1,1};
	} m_BoxParams;
	struct CapsuleParams
	{
		MathLib::HReal m_Radius=1;
		MathLib::HReal m_HalfHeight=1;
	} m_CapsuleParams;
	struct PlaneParams
	{
		MathLib::HVector3 m_Normal = {0,1,0};
		MathLib::HReal m_Distance=0;
	} m_PlaneParams;
	struct TriangleMeshParams
	{
		std::vector<MathLib::HVector3> m_Vertices;
		std::vector<uint32_t> m_Indices;
	} m_TriangleMeshParams;
	struct ConvexMeshParams
	{
		std::vector<MathLib::HVector3> m_Vertices;
	} m_ConvexMeshParams;
	MathLib::HVector3 m_Scale = { 1,1,1 };
};

enum class PhysicsObjectType
{
	PHYSICS_OBJECT_TYPE_RIGID_STATIC,
	PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC,
	PHYSICS_OBJECT_TYPE_COUNT
};

enum class PhysicsObjectAttributeType
{
	eRigidStatic,
	eRigidDynamic,
	eRigidKinematic,
};

struct PhysicsObjectCreateOptions
{
	PhysicsObjectType m_ObjectType;
	MathLib::HTransform3 m_Transform;
	PhysicsMaterialCreateOptions m_MaterialOptions;
};

struct PhysicsMeshData
{
	std::vector<MathLib::HVector3> m_Vertices;
	std::vector<uint32_t> m_Indices;
};

struct ConvexDecomposeOptions
{
	uint32_t m_MaximumNumberOfHulls = 8;  // Maximum number of convex hull generated
	uint32_t m_MaximumNumberOfVerticesPerHull = 64;  // (default=64, range=4-1024)
	uint32_t m_VoxelGridResolution = 1000000;        //(default=1,000,000, range=10,000-16,000,000).
	MathLib::HReal m_Concavity = 0.0025f;                     // Value between 0 and 1
};