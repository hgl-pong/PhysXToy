#pragma once
#include "Physics/PhysicsCommon.h"

class PhysicsConvexUtils
{
public:
	static void BuildConvexMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices, PhysicsMeshData& meshdata);
	static void DecomposeConvexMesh(const PhysicsMeshData& meshdata, std::vector<PhysicsMeshData>& convexMeshes);
};