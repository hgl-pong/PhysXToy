#include "Physics/PhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "PxBoxGeometry.h"
using namespace physx;
inline bool CreatePhysXGeometry(PhysicsEngine* engine, const CollisionGeometryCreateOptions &options, PxGeometry **geometry)
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
		const CollisionGeometryCreateOptions::SphereParams&sphereOptions = options.m_SphereParams;
		*geometry = new PxSphereGeometry(sphereOptions.m_Radius);
		return true;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
	{
		const CollisionGeometryCreateOptions::CapsuleParams&capsuleOptions = options.m_CapsuleParams;
		*geometry = new PxCapsuleGeometry(capsuleOptions.m_Radius, capsuleOptions.m_HalfHeight);
		return true;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
	{
		//const TriangleMeshGeometryCreateOptions &triangleMeshOptions = static_cast<const TriangleMeshGeometryCreateOptions &>(options);
		//*geometry = new PxTriangleMeshGeometry(triangleMeshOptions.triangleMesh);
		return true;
	}
	case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
	{
		//const ConvexMeshGeometryCreateOptions &convexMeshOptions = static_cast<const ConvexMeshGeometryCreateOptions &>(options);
		//*geometry = new PxConvexMeshGeometry(convexMeshOptions.convexMesh);
		return true;
	}
	default:
		return false;
	}
}

inline bool CreatePhysXMaterial(PhysicsEngine* engine,const PhysicsMaterialCreateOptions &options, PxMaterial **material)
{
	if (material == nullptr)
		return false;
	*material = engine->m_Physics->createMaterial(options.m_StaticFriction, options.m_DynamicFriction, options.m_Restitution);
	return true;
}