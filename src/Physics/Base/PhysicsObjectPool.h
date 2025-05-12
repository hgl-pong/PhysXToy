#pragma once
#include <vector>
#include <queue>
#include <mutex>
#include <cassert>

template <class PhysicsObject>
class PhysicsObjectPool
{
public:
	PhysicsObjectPool(uint32_t maxSize = 500)
		: m_MaxSize(maxSize)
	{
		for (uint32_t i = 0; i < maxSize; ++i)
		{
			PhysicsObject* object = new PhysicsObject();
			if (object && object->IsValid())
			{
				m_ObjectPool.push(object);
			}
			else
			{
				delete object;
				break;
			}
		}
	}
	
	~PhysicsObjectPool()
	{
		while (!m_ObjectPool.empty())
		{
			PhysicsObject* object = m_ObjectPool.front();
			m_ObjectPool.pop();
			if (object)
			{
				if (object->IsValid())
				{
					object->Release();
				}
				delete object;
			}
		}
	}
	
	PhysicsObject* TakeObject()
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		if (m_ObjectPool.empty())
		{
			PhysicsObject* newObject = new PhysicsObject();
			if (newObject && newObject->IsValid())
			{
				return newObject;
			}
			else
			{
				delete newObject;
				return nullptr;
			}
		}
		
		PhysicsObject* object = m_ObjectPool.front();
		m_ObjectPool.pop();
		return object;
	}

	void ReturnObject(PhysicsObject** object)
	{
		if (!object || !(*object))
			return;
			
		std::lock_guard<std::mutex> lock(m_Mutex);
		if (m_ObjectPool.size() < m_MaxSize && (*object)->IsValid())
		{
			m_ObjectPool.push(*object);
		}
		else
		{
			if ((*object)->IsValid())
			{
				(*object)->Release();
			}
			delete *object;
		}
		*object = nullptr;
	}
	
	void Resize(uint32_t newMaxSize)
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		if (newMaxSize < m_MaxSize)
		{
			while (m_ObjectPool.size() > newMaxSize)
			{
				PhysicsObject* object = m_ObjectPool.front();
				m_ObjectPool.pop();
				if (object)
				{
					if (object->IsValid())
					{
						object->Release();
					}
					delete object;
				}
			}
		}
		else if (newMaxSize > m_MaxSize)
		{
			size_t currentSize = m_ObjectPool.size();
			uint32_t additionalObjects = newMaxSize - m_MaxSize;
			
			for (uint32_t i = 0; i < additionalObjects; ++i)
			{
				PhysicsObject* object = new PhysicsObject();
				if (object && object->IsValid())
				{
					m_ObjectPool.push(object);
				}
				else
				{
					delete object;
					break;
				}
			}
		}
		
		m_MaxSize = newMaxSize;
	}
	
	size_t Size() const
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		return m_ObjectPool.size();
	}
	
	uint32_t MaxSize() const
	{
		return m_MaxSize;
	}
	
	void Clear()
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		while (!m_ObjectPool.empty())
		{
			PhysicsObject* object = m_ObjectPool.front();
			m_ObjectPool.pop();
			if (object)
			{
				if (object->IsValid())
				{
					object->Release();
				}
				delete object;
			}
		}
	}
	
private:
	mutable std::mutex m_Mutex;
	std::queue<PhysicsObject*> m_ObjectPool;
	uint32_t m_MaxSize;
};