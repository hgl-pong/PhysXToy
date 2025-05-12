#pragma once
#include <Physics/PhysicsCommon.h>

PHYSICS_INLINE MathLib::HAABBox3D ComputeBoundingBox(IPhysicsObject *physicsObject)
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

PHYSICS_INLINE bool IsSphericalLimitEnabled(const PhysicsLimits& limits)	{ return limits.m_MinValue>0.0f && limits.m_MaxValue>0.0f;	}
PHYSICS_INLINE void SetSphericalLimitDisabled(PhysicsLimits& limits)		{ limits.Set(-1.0f, -1.0f);									}

PHYSICS_INLINE bool IsHingeLimitEnabled(const PhysicsLimits& limits)		{ return limits.m_MinValue<=limits.m_MaxValue;	}
PHYSICS_INLINE void SetHingeLimitDisabled(PhysicsLimits& limits)			{ limits.Set(1.0f, -1.0f);						}

PHYSICS_INLINE bool IsPrismaticLimitEnabled(const PhysicsLimits& limits)	{ return limits.m_MinValue<=limits.m_MaxValue;	}
PHYSICS_INLINE void SetPrismaticLimitDisabled(PhysicsLimits& limits)		{ limits.Set(1.0f, -1.0f);						}

PHYSICS_INLINE bool IsMinDistanceLimitEnabled(const PhysicsLimits& limits)	{ return limits.m_MinValue>=0.0f;	}
PHYSICS_INLINE bool IsMaxDistanceLimitEnabled(const PhysicsLimits& limits)	{ return limits.m_MaxValue>=0.0f;	}
PHYSICS_INLINE void SetMinDistanceLimitDisabled(PhysicsLimits& limits)		{ limits.m_MinValue = -1.0f;			}
PHYSICS_INLINE void SetMaxDistanceLimitDisabled(PhysicsLimits& limits)		{ limits.m_MaxValue = -1.0f;			}

PHYSICS_INLINE bool IsD6LinearLimitEnabled(const PhysicsLimits& limits)	{ return limits.m_MinValue<=limits.m_MaxValue;	}
PHYSICS_INLINE void SetD6LinearLimitDisabled(PhysicsLimits& limits)		{ limits.Set(1.0f, -1.0f);						}


namespace PhysicsCacheUtils
{
    PhysicsPtr<IColliderGeometry> CreateColliderGeometry(const CollisionGeometryCreateOptions& options);

	void CleanupUnusedGeometries();

	size_t GetGeometryCacheSize();

	void ClearGeometryCache();
}
