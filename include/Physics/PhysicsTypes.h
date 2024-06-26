#pragma once
#include <Math/MathUtils.h>
#include <Math/GraphicUtils/MeshData.h>
#define DEFAULT_CPU_DISPATCHER_NUM_THREADS 2
#define DEFAULT_SOLVER_ITERATION_COUNT 6

template <typename T>
struct PhysicsDeleter
{
	void operator()(T *ptr) const
	{
		if (ptr)
		{
			ptr->Release();
			ptr = nullptr;
		}
	}
};

template <typename T>
using PhysicsPtr = std::shared_ptr<T>;

template <typename T>
PhysicsPtr<T> make_physics_ptr(T *ptr)
{
	return PhysicsPtr<T>(ptr, PhysicsDeleter<T>());
}

template <typename T>
struct PhysXDeleter
{
	void operator()(T *ptr) const
	{
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
PhysXPtr<T> make_physx_ptr(T *ptr)
{
	return PhysXPtr<T>(ptr);
}

struct PhysicsEngineOptions
{
	uint32_t m_NumThreads = DEFAULT_CPU_DISPATCHER_NUM_THREADS;
	bool m_bEnablePVD = true;
	uint32_t m_SolverIterationCount = DEFAULT_SOLVER_ITERATION_COUNT;
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
	MathLib::HReal m_StaticFriction = 0.5;
	MathLib::HReal m_DynamicFriction = 0.5;
	MathLib::HReal m_Restitution = 0.6;
	MathLib::HReal m_Density = 10;
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
		MathLib::HReal m_Radius = 1;
	} m_SphereParams;
	struct BoxParams
	{
		MathLib::HVector3 m_HalfExtents = {1, 1, 1};
	} m_BoxParams;
	struct CapsuleParams
	{
		MathLib::HReal m_Radius = 1;
		MathLib::HReal m_HalfHeight = 1;
	} m_CapsuleParams;
	struct PlaneParams
	{
		MathLib::HVector3 m_Normal = {0, 1, 0};
		MathLib::HReal m_Distance = 0;
	} m_PlaneParams;
	struct TriangleMeshParams
	{
		std::vector<MathLib::HVector3> m_Vertices;
		std::vector<uint32_t> m_Indices;
	} m_TriangleMeshParams;
	struct ConvexMeshParams
	{
		std::vector<MathLib::HVector3> m_Vertices;
		std::vector<uint32_t> m_Indices;
	} m_ConvexMeshParams;
	MathLib::HVector3 m_Scale = {1, 1, 1};
};

enum class PhysicsObjectType
{
	PHYSICS_OBJECT_TYPE_RIGID_STATIC,
	PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC,
	PHYSICS_OBJECT_TYPE_COUNT
};

struct PhysicsObjectCreateOptions
{
	PhysicsObjectType m_ObjectType;
	MathLib::HTransform3 m_Transform;
	PhysicsMaterialCreateOptions m_MaterialOptions;
};

typedef MathLib::GraphicUtils::MeshData32 PhysicsMeshData;

struct ConvexDecomposeOptions
{
	uint32_t m_MaximumNumberOfHulls = 8;			// Maximum number of convex hull generated
	uint32_t m_MaximumNumberOfVerticesPerHull = 64; // (default=64, range=4-1024)
	uint32_t m_VoxelGridResolution = 1000000;		//(default=1,000,000, range=10,000-16,000,000).
	MathLib::HReal m_Concavity = 0.0025f;			// Value between 0 and 1
};