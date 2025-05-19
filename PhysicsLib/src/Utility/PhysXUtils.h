#pragma once
#include <PxPhysicsAPI.h>
#include "Physics/PhysicsTypes.h"
#include <memory>

PHYSICS_INLINE physx::PxSimulationFilterShader GetFilterShader(const PhysicsSceneFilterShaderType& type)
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
	PHYSICS_INLINE physx::PxVec3 ToPx(const MathLib::HVector3& vector)
	{
		return physx::PxVec3(vector[0],vector[1],vector[2]);
	}

	PHYSICS_INLINE MathLib::HVector3 FromPx(const physx::PxVec3& vector)
	{
		return MathLib::HVector3(vector.x, vector.y, vector.z);
	}

	PHYSICS_INLINE physx::PxTransform ToPx(const MathLib::HTransform3& transform)
	{
		MathLib::HVector3 translation = transform.translation();
		MathLib::HMatrix3 rotationMatrix = transform.rotation();

		MathLib::HQuaternion rotationQuaternion(rotationMatrix);

		physx::PxVec3 pxTranslation(translation.x(), translation.y(), translation.z());
		physx::PxQuat pxRotation(rotationQuaternion.x(), rotationQuaternion.y(), rotationQuaternion.z(), rotationQuaternion.w());

		return physx::PxTransform(pxTranslation, pxRotation);
	}

	PHYSICS_INLINE MathLib::HTransform3 FromPx(const physx::PxTransform& pxTransform)
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

	PHYSICS_INLINE MathLib::HAABBox3D FromPx(const physx::PxBounds3& bounds)
	{
		return MathLib::HAABBox3D(FromPx(bounds.minimum), FromPx(bounds.maximum));
	}

	PHYSICS_INLINE physx::PxBounds3 ToPx(const MathLib::HAABBox3D& bounds)
	{
		return physx::PxBounds3(ToPx(bounds.min()), ToPx(bounds.max()));
	}

	PHYSICS_INLINE MathLib::HMatrix3 FromPx(const physx::PxMat33& mat)
	{
		MathLib::HMatrix3 result;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				result(i, j) = mat(i, j);
		return result;
	}

	PHYSICS_INLINE physx::PxMat33 ToPx(const MathLib::HMatrix3& mat)
	{
		physx::PxMat33 result;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				result(i, j) = mat(i, j);
		return result;
	}

	PHYSICS_INLINE void PxLocationHitToRaycastHit(const physx::PxLocationHit& hit, PhysicsRaycastHit& outHit)
	{
		outHit.m_Position = MathLib::HVector3(hit.position.x, hit.position.y, hit.position.z);
		outHit.m_Normal = MathLib::HVector3(hit.normal.x, hit.normal.y, hit.normal.z);
		outHit.m_Distance = hit.distance;
	}

	PHYSICS_INLINE void FromPx(const physx::PxRaycastHit& hit, PhysicsRaycastHit& outHit)
	{
		PxLocationHitToRaycastHit(hit, outHit);
	}

	PHYSICS_INLINE void FromPx(const physx::PxSweepHit& hit, PhysicsRaycastHit& outHit)
	{
		PxLocationHitToRaycastHit(hit, outHit);

		// For shapes that overlap with the query primitive at the origin PhysX returns undefined position.
		// Here we fix it by setting the position to zero.
		// Note, the normal is not undefined, it's actually opposite to the query direction.
		// Refer to PhysX Guide, Scene Queries section, around "Sweeps with Initial Overlap".
		if (hit.hadInitialOverlap())
		{
			outHit.m_Position.setZero();
		}
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
		meshDesc.points.data = reinterpret_cast<const physx::PxVec3*>(verts);

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

PHYSICS_INLINE physx::PxBounds3 CalculateBoundingBox(physx::PxRigidActor* actor) {
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

PHYSICS_INLINE void NormalToTangents(const physx::PxVec3& n, physx::PxVec3& t1, physx::PxVec3& t2)
{
	const physx::PxReal m_sqrt1_2 = physx::PxReal(0.7071067811865475244008443621048490);
	if(fabsf(n.z) > m_sqrt1_2)
	{
		const physx::PxReal a = n.y*n.y + n.z*n.z;
		const physx::PxReal k = physx::PxReal(1.0)/physx::PxSqrt(a);
		t1 = physx::PxVec3(0,-n.z*k,n.y*k);
		t2 = physx::PxVec3(a*k,-n.x*t1.z,n.x*t1.y);
	}
	else 
	{
		const physx::PxReal a = n.x*n.x + n.y*n.y;
		const physx::PxReal k = physx::PxReal(1.0)/physx::PxSqrt(a);
		t1 = physx::PxVec3(-n.y*k,n.x*k,0);
		t2 = physx::PxVec3(-n.z*t1.y,n.z*t1.x,a*k);
	}
	t1.normalize();
	t2.normalize();
}

PHYSICS_INLINE physx::PxQuat ComputeJointQuat(const physx::PxTransform* pose, const physx::PxVec3& localAxis)
{
	physx::PxVec3 axisw = pose ? pose->rotate(localAxis) : localAxis;
	axisw.normalize();

	physx::PxVec3 normalw, binormalw;
	NormalToTangents(axisw, binormalw, normalw);

	const physx::PxVec3 localNormal = pose ? pose->rotateInv(normalw) : normalw;

	const physx::PxMat33 rot(localAxis, localNormal, localAxis.cross(localNormal));
	physx::PxQuat q(rot);
	q.normalize();

	return q;
}
