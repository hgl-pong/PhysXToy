#pragma once
#include "Physics/PhysicsCommon.h"
#include "Base/ObjectCache.h"
#include "Utility/PhysicsUtils.h"

extern bool AreGeometriesEqual(const CollisionGeometryCreateOptions& a, const CollisionGeometryCreateOptions& b);
extern size_t GenerateHash(const CollisionGeometryCreateOptions& options);

namespace PhysicsBase
{
    template<>
    struct Creator<CollisionGeometryCreateOptions, IColliderGeometry>
    {
        PhysicsPtr<IColliderGeometry> Create(const CollisionGeometryCreateOptions& options)
        {
            return PhysicsCacheUtils::CreateColliderGeometry(options);
        }
    };
}

namespace std
{
    template <>
    struct hash<CollisionGeometryCreateOptions> {
        size_t operator()(const CollisionGeometryCreateOptions& options) const
        {
            return GenerateHash(options);
        }
    };

    template<>
    struct equal_to<CollisionGeometryCreateOptions>
    {
        bool operator()(const CollisionGeometryCreateOptions& a, const CollisionGeometryCreateOptions& b) const
        {
            return AreGeometriesEqual(a, b);
        }
    };
}

using ColliderGeometryCache = ObjectCache<IColliderGeometry, CollisionGeometryCreateOptions>;
