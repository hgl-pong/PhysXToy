#pragma once
using namespace physx;
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

		PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

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