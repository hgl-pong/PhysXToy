#pragma once
#include "Physics/PhysicsCommon.h"

namespace std
{
	template <>
	struct hash<PhysicsMaterialCreateOptions>
	{
		size_t operator()(const PhysicsMaterialCreateOptions& material) const
		{
			return hash<MathLib::HReal>()(material.m_Restitution) ^ hash<MathLib::HReal>()(material.m_StaticFriction) ^ hash<MathLib::HReal>()(material.m_DynamicFriction) ^ hash<MathLib::HReal>()(material.m_Density);
		}
	};

	template <>
	struct equal_to<PhysicsMaterialCreateOptions>
	{
		bool operator()(const PhysicsMaterialCreateOptions& lhs, const PhysicsMaterialCreateOptions& rhs) const
		{
			return lhs.m_Restitution == rhs.m_Restitution && lhs.m_StaticFriction == rhs.m_StaticFriction && lhs.m_DynamicFriction == rhs.m_DynamicFriction&& lhs.m_Density == rhs.m_Density;
		}
	};
};

namespace physx
{
	class PxMaterial;
}

class PhysicsMaterialManager
{
public:
private:
	struct MaterialInfo
	{
		physx::PxMaterial* m_Material = nullptr;
		uint32_t m_ReferenceCount;
	};
	std::unordered_map<PhysicsMaterialCreateOptions,MaterialInfo > m_Materials;
};