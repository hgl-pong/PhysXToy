#pragma once
#include <PxPhysicsAPI.h>
#include "Physics/PhysicsTypes.h"

inline physx::PxSimulationFilterShader GetFilterShader(const PhysicsSceneFilterShaderType& type)
{
	switch (type)
	{
	case PhysicsSceneFilterShaderType::eDEFAULT:
	{
		return physx::PxDefaultSimulationFilterShader;
		break;
	}
	default:
		break;
	}
	return nullptr;
}

namespace ConvertUtils
{
	inline physx::PxVec3 ToPx(const MathLib::HVector3& vector)
	{
		return physx::PxVec3(vector[0],vector[1],vector[2]);
	}

	inline MathLib::HVector3 FromPx(const physx::PxVec3& vector)
	{
		return MathLib::HVector3(vector.x, vector.y, vector.z);
	}

	inline physx::PxTransform ToPx(const MathLib::HTransform3& transform)
	{
		MathLib::HVector3 translation = transform.translation();

		MathLib::HMatrix3 rotationMatrix = transform.rotation();
		MathLib::HQuaternion rotationQuaternion(rotationMatrix);

		physx::PxVec3 pxTranslation(translation.x(), translation.y(), translation.z());
		physx::PxQuat pxRotation(rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z(), rotationQuaternion.w());

		return physx::PxTransform(pxTranslation, pxRotation);
	}

	inline MathLib::HTransform3 FromPx(const physx::PxTransform& pxTransform)
	{
		physx::PxVec3 pxTranslation = pxTransform.p;
		physx::PxQuat pxRotation = pxTransform.q;

		MathLib::HVector3 translation(pxTranslation.x, pxTranslation.y, pxTranslation.z);
		MathLib::HQuaternion rotation(pxRotation.w, pxRotation.x, pxRotation.y, pxRotation.z);

		MathLib::HTransform3 eigenTransform = MathLib::HTransform3::Identity();
		eigenTransform.translate(translation);
		eigenTransform.rotate(rotation);

		return eigenTransform;
	}

	inline MathLib::HAABBox3D FromPx(const physx::PxBounds3& bounds)
	{
		return MathLib::HAABBox3D(FromPx(bounds.minimum), FromPx(bounds.maximum));
	}

	inline physx::PxBounds3 ToPx(const MathLib::HAABBox3D& bounds)
	{
		return physx::PxBounds3(ToPx(bounds.min()), ToPx(bounds.max()));
	}
};

namespace PhysXConstructTools
{
	template <bool directInsertion, uint32_t gaussMapLimit>
	inline physx::PxConvexMesh* CreatePxConvexMesh(uint32_t numVerts, const MathLib::HVector3* verts)
	{
		if (PhysicsEngineUtils::GetPhysicsEngine() == nullptr)
			return nullptr;
		physx::PxTolerancesScale tolerances;
		physx::PxCookingParams params(tolerances);

		// Use the new (default) PxConvexMeshCookingType::eQUICKHULL
		params.convexMeshCookingType = physx::PxConvexMeshCookingType::eQUICKHULL;

		// If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
		// If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
		params.gaussMapLimit = gaussMapLimit;

		physx::PxConvexMeshDesc desc;

		desc.points.data = verts;
		desc.points.count = numVerts;
		desc.points.stride = sizeof(physx::PxVec3);
		desc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;

		physx::PxU32 meshSize = 0;
		physx::PxConvexMesh* convex = nullptr;

		if (directInsertion)
		{
			convex = PxCreateConvexMesh(params, desc, PxGetPhysics().getPhysicsInsertionCallback());
			PX_ASSERT(convex);
		}
		else
		{
			physx::PxDefaultMemoryOutputStream outStream;
			bool res = PxCookConvexMesh(params, desc, outStream);
			PX_UNUSED(res);
			PX_ASSERT(res);
			meshSize = outStream.getSize();

			physx::PxDefaultMemoryInputData inStream(outStream.getData(), outStream.getSize());
			convex = PxGetPhysics().createConvexMesh(inStream);
			PX_ASSERT(convex);
		}
		return convex;
	}

	template <bool directInsertion>
	inline physx::PxTriangleMesh* CreatePxTriangleMesh(uint32_t numVerts, const MathLib::HVector3* verts, uint32_t numTris, const uint32_t* tris)
	{
		if (PhysicsEngineUtils::GetPhysicsEngine() == nullptr)
			return nullptr;

		physx::PxTriangleMeshDesc meshDesc;
		meshDesc.points.count = numVerts;
		meshDesc.points.stride = sizeof(physx::PxVec3);
		meshDesc.points.data = reinterpret_cast<const physx::PxVec3*>(verts); // ÀàÐÍ×ª»»

		meshDesc.triangles.count = numTris;
		meshDesc.triangles.stride = 3 * sizeof(physx::PxU32);
		meshDesc.triangles.data = tris;

		physx::PxTriangleMesh* triMesh = nullptr;
		physx::PxTolerancesScale scale;
		physx::PxCookingParams params(scale);

		if (directInsertion)
		{
			triMesh = PxCreateTriangleMesh(params, meshDesc, PxGetPhysics().getPhysicsInsertionCallback());
			if (!triMesh)
			{
				return nullptr;
			}
		}
		else
		{
			physx::PxDefaultMemoryOutputStream outBuffer;
			if (!PxCookTriangleMesh(params, meshDesc, outBuffer))
			{
				return nullptr;
			}

			physx::PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
			triMesh = PxGetPhysics().createTriangleMesh(stream);
			if (!triMesh)
			{
				return nullptr;
			}
		}

		return triMesh;
	}
}

inline physx::PxBounds3 CalculateBoundingBox(physx::PxRigidActor* actor) {
	physx::PxU32 numShapes = actor->getNbShapes();
	std::vector<physx::PxShape*>shapes(numShapes);
	actor->getShapes(shapes.data(), numShapes);

	physx::PxBounds3 bounds = physx::PxBounds3::empty();

	for (physx::PxU32 i = 0; i < numShapes; i++) {
		physx::PxShape* shape = shapes[i];

		physx::PxTransform localPose = shape->getLocalPose();
		physx::PxBounds3 localBounds;
		physx::PxGeometryQuery::computeGeomBounds(localBounds, shape->getGeometry(), localPose);

		//physx::PxTransform globalPose = actor->getGlobalPose() * localPose;
		//physx::PxBounds3 worldBounds;
		//physx::PxGeometryQuery::computeGeomBounds(worldBounds, shape->getGeometry(), globalPose);

		bounds.include(localBounds);
	}

	return bounds;
}
