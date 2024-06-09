#pragma once
#include <Physics/PhysicsCommon.h>

MathLib::HAABBox3D ComputeBoundingBox(IPhysicsObject *physicsObject)
{
	MathLib::HAABBox3D newBox;
	std::vector<PhysicsPtr<IColliderGeometry>> colliderGeometries;
	std::vector<MathLib::HTransform3> geoLocalPos;
	physicsObject->GetColliderGeometries(colliderGeometries, &geoLocalPos);
	for (size_t i = 0; i < colliderGeometries.size(); i++)
	{
		MathLib::HAABBox3D box = colliderGeometries[i]->GetBoundingBox();
		MathLib::HTransform3 trans = geoLocalPos[i];
		box.transform(trans);
		newBox.extend(box);
	}
	return newBox;
}