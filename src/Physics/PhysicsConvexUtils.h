#pragma once
#include "Physics/PhysicsCommon.h"
#include "PhysXUtils.h"
namespace PhysicsConvexUtils
{
	static void BuildConvexMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices, PhysicsMeshData& meshdata)
	{
		{
			if (PhysicsEngineUtils::GetPhysicsEngine() == nullptr)
				return;

			physx::PxConvexMesh* convexMesh = PhysXConstructTools::CreatePxConvexMesh<true, 256>(vertices.size(), vertices.data());

			const physx::PxU32 nbPolys = convexMesh->getNbPolygons();
			const physx::PxU8* polygons = convexMesh->getIndexBuffer();
			const physx::PxVec3* verts = convexMesh->getVertices();
			physx::PxU32 nbVerts = convexMesh->getNbVertices();
			PX_UNUSED(nbVerts);

			meshdata.m_Vertices.reserve(nbVerts);
			for (physx::PxU32 i = 0; i < nbVerts; i++)
			{
				meshdata.m_Vertices.push_back(MathLib::HVector3(verts[i].x, verts[i].y, verts[i].z));
			}

			physx::PxU32 numTotalTriangles = 0;
			for (physx::PxU32 i = 0; i < nbPolys; i++)
			{
				physx::PxHullPolygon data;
				convexMesh->getPolygonData(i, data);

				const physx::PxU32 nbTris = physx::PxU32(data.mNbVerts - 2);
				const physx::PxU8 vref0 = polygons[data.mIndexBase + 0];
				PX_ASSERT(vref0 < nbVerts);
				meshdata.m_Indices.reserve(nbTris * 3);
				for (physx::PxU32 j = 0; j < nbTris; j++)
				{
					const physx::PxU32 vref1 = polygons[data.mIndexBase + 0 + j + 1];
					const physx::PxU32 vref2 = polygons[data.mIndexBase + 0 + j + 2];

					// generate face normal:
					physx::PxVec3 e0 = verts[vref1] - verts[vref0];
					physx::PxVec3 e1 = verts[vref2] - verts[vref0];

					PX_ASSERT(vref1 < nbVerts);
					PX_ASSERT(vref2 < nbVerts);

					meshdata.m_Indices.push_back(vref0);
					meshdata.m_Indices.push_back(vref1);
					meshdata.m_Indices.push_back(vref2);
					numTotalTriangles++;
				}
			}
			PX_RELEASE(convexMesh);
		}
	}
};