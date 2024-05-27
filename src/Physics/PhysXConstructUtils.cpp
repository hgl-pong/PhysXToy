#include "Physics/PhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "PxBoxGeometry.h"
#include "PhysXUtils.h"
using namespace physx;
inline bool CreatePhysXGeometry(PhysicsEngine *engine, const CollisionGeometryCreateOptions &options, PxGeometry **geometry)
{
	if (geometry == nullptr)
		return false;
	switch (options.m_GeometryType)
	{
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
	{
		const CollisionGeometryCreateOptions::BoxParams &boxOptions = options.m_BoxParams
																		  *geometry = new PxBoxGeometry(boxOptions.m_HalfExtents, boxOptions.m_HalfExtents, boxOptions.m_HalfExtents);
		return true;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
	{
		const CollisionGeometryCreateOptions::SphereParams &sphereOptions = options.m_SphereParams;
		*geometry = new PxSphereGeometry(sphereOptions.m_Radius);
		return true;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
	{
		const CollisionGeometryCreateOptions::CapsuleParams &capsuleOptions = options.m_CapsuleParams;
		*geometry = new PxCapsuleGeometry(capsuleOptions.m_Radius, capsuleOptions.m_HalfHeight);
		return true;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
	{
		const CollisionGeometryCreateOptions::TriangleMeshParams &triangleMeshOptions = static_cast<const CollisionGeometryCreateOptions::TriangleMeshParams &>(options);
		const PxTriangleMesh *triangleMesh = PhysXConstructTools::CreateTriangleMesh(engine->m_Physics, triangleMeshOptions.m_Vertices, triangleMeshOptions.m_iNumVertices, triangleMeshOptions.m_Indices, triangleMeshOptions.m_iNumIndices);
		*geometry = new PxTriangleMeshGeometry(triangleMesh);
		triangleMesh->release();
		return true;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
	{
		const CollisionGeometryCreateOptions::ConvexMeshParams &convexMeshOptions = static_cast<const CollisionGeometryCreateOptions::ConvexMeshParams &>(options);
		const PxConvexMesh *convexMesh = PhysXConstructTools::CreateConvexMesh<true, 256>(engine->m_Physics, convexMeshOptions.m_Vertices, convexMeshOptions.m_iNumVertices);
		*geometry = new PxConvexMeshGeometry(convexMesh);
		convexMesh->release();
		return true;
	}
	default:
		return false;
	}
}

inline bool CreatePhysXMaterial(PhysicsEngine *engine, const PhysicsMaterialCreateOptions &options, PxMaterial **material)
{
	if (material == nullptr)
		return false;
	*material = engine->m_Physics->createMaterial(options.m_StaticFriction, options.m_DynamicFriction, options.m_Restitution);
	return true;
}