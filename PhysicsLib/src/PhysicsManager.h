#pragma once

#include "Physics/PhysicsCommon.h"

template<class PhysicsResource, class ResourceCreateParams,class Hasher = std::hash<ResourceCreateParams>,class KeyEqual=std::equal_to<ResourceCreateParams>>
class PhysicsResourceManager
{
public:
	PhysicsResourceManager() = default;
	~PhysicsResourceManager() = default;

	PhysicsPtr<PhysicsResource> GetResource(const ResourceCreateParams &params)
	{
		auto it = m_Resources.find(params);
		if (it != m_Resources.end())
		{
			return it->second;
		}
		return nullptr;
	}

	PhysicsPtr<PhysicsResource> CreateResource(const ResourceCreateParams &params)
	{
		auto it = m_Resources.find(params);
		if (it != m_Resources.end())
		{
			return it->second;
		}
		PhysicsPtr<PhysicsResource> resource = std::make_shared<PhysicsResource>(params);
		m_Resources[params] = resource;
		return resource;
	}

	void RemoveResource(const ResourceCreateParams &params)
	{
		auto it = m_Resources.find(params);
		if (it != m_Resources.end())
		{
			m_Resources.erase(it);
		}
	}

	void Clear()
	{
		m_Resources.clear();
	}

private:
	std::unordered_map<ResourceCreateParams,PhysicsPtr<PhysicsResource>,Hasher,KeyEqual> m_Resources;
};