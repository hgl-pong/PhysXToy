#pragma once
#include <vector>
#include <queue>
#include <mutex>
template <class PhysicsObject>
class PhysicsObjectPool
{
public:
	PhysicsObjectPool(uint32_t maxSize = 500)
	{
		m_MaxSize = maxSize;
		m_ObjectPool.resize(maxSize);
		for (auto& object : m_ObjectPool)
		{
			object = new PhysicsObject();
		}
	}
	
	PhysicsObject* TakeObject()
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		PhysicsObject* object = m_ObjectPool.front();
		m_ObjectPool.pop();
		if (m_ObjectPool.size() / m_MaxSize < 0.5)
		{
			m_ObjectPool.resize(m_MaxSize);
		}
		return object;
	}

	void ReturnObject(PhysicsObject** object)
	{
		std::lock_guard<std::mutex> lock(m_Mutex);
		if (m_ObjectPool.size() < m_MaxSize)
		{
			m_ObjectPool.push(*object);
		}
		else
		{
			if (object)
			{
				*object->Release();
				delete *object;
			}
		}
		*object = nullptr;
	}
private:
	std::mutex m_Mutex;
	std::queue<PhysicsObject*> m_ObjectPool;
	uint32_t m_MaxSize;
};