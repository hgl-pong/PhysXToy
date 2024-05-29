#pragma once
using namespace physx;
namespace ConvertUtils
{
	physx::PxVec3 ToPxVec3(const MathLib::HVector3& vector)
	{
		return physx::PxVec3(vector[0],vector[1],vector[2]);
	}

	MathLib::HVector3 FromPxVec3(const physx::PxVec3& vector)
	{
		return MathLib::HVector3(vector.x, vector.y, vector.z);
	}

	physx::PxTransform ToPxTransform(const MathLib::HTransform3& transform)
	{
		MathLib::HVector3 translation = transform.translation();

		MathLib::HMatrix3 rotationMatrix = transform.rotation();
		MathLib::HQuaternion rotationQuaternion(rotationMatrix);

		PxVec3 pxTranslation(translation.x(), translation.y(), translation.z());
		PxQuat pxRotation(rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z(), rotationQuaternion.w());

		return PxTransform(pxTranslation, pxRotation);
	}

	MathLib::HTransform3 FromPxTransform(const physx::PxTransform& pxTransform)
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
};

namespace PhysXConstructTools
{
	template <bool directInsertion, PxU32 gaussMapLimit>
	inline PxConvexMesh* CreateConvexMesh(PxPhysics* gPhysics, PxU32 numVerts, const PxVec3* verts)
	{
		PxTolerancesScale tolerances;
		PxCookingParams params(tolerances);

		// Use the new (default) PxConvexMeshCookingType::eQUICKHULL
		params.convexMeshCookingType = PxConvexMeshCookingType::eQUICKHULL;

		// If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
		// If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
		params.gaussMapLimit = gaussMapLimit;

		// Setup the convex mesh descriptor
		PxConvexMeshDesc desc;

		// We provide points only, therefore the PxConvexFlag::eCOMPUTE_CONVEX flag must be specified
		desc.points.data = verts;
		desc.points.count = numVerts;
		desc.points.stride = sizeof(PxVec3);
		desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

		PxU32 meshSize = 0;
		PxConvexMesh* convex = NULL;

		if (directInsertion)
		{
			// Directly insert mesh into PhysX
			convex = PxCreateConvexMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
			PX_ASSERT(convex);
		}
		else
		{
			// Serialize the cooked mesh into a stream.
			PxDefaultMemoryOutputStream outStream;
			bool res = PxCookConvexMesh(params, desc, outStream);
			PX_UNUSED(res);
			PX_ASSERT(res);
			meshSize = outStream.getSize();

			// Create the mesh from a stream.
			PxDefaultMemoryInputData inStream(outStream.getData(), outStream.getSize());
			convex = gPhysics->createConvexMesh(inStream);
			PX_ASSERT(convex);
		}
		return convex;
	}

	inline PxTriangleMesh* CreateTriangleMesh(PxPhysics* gPhysics, PxU32 numVerts, const PxVec3* verts, PxU32 numTris, const PxU32* tris)
	{
		PxTriangleMeshDesc meshDesc;
		meshDesc.points.count = numVerts;
		meshDesc.points.stride = sizeof(PxVec3);
		meshDesc.points.data = verts;

		meshDesc.triangles.count = numTris;
		meshDesc.triangles.stride = 3 * sizeof(PxU32);
		meshDesc.triangles.data = tris;

		PxTriangleMesh* aTriangleMesh = NULL;
		PxDefaultMemoryOutputStream writeBuffer;
		PxTriangleMeshCookingResult::Enum result;

		PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
		aTriangleMesh = gPhysics->createTriangleMesh(readBuffer);
		return aTriangleMesh;
	}
}