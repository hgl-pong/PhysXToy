#pragma once
#include <unordered_map>
#include <vector>
#include <functional>

namespace PhysicsBase
{
    template<typename KeyT, typename T>
    struct Creator
    {
        PhysicsPtr<T> operator()(const typename KeyT& options)
        {
            return nullptr;
        }
    };
}

template<typename T, typename KeyT, class Creator = PhysicsBase::Creator<KeyT, T>, class Hasher = std::hash<KeyT>, class Comparer = std::equal_to<KeyT>>
class ObjectCache
{
public:
    
    static ObjectCache<T, KeyT, Creator, Hasher, Comparer>& GetInstance()
    {
        static ObjectCache<T, KeyT, Creator, Hasher, Comparer> instance;
        return instance;
    }
    
    PhysicsPtr<T> GetOrCreate(
        const KeyT& key)
    {
        auto it = m_Cache.find(key);
        if (it != m_Cache.end())
        {
            return it->second;
        }
        
        PhysicsPtr<T> newObject = m_Creator.Create(key);
        if (newObject)
        {
            m_Cache[key] = newObject;
        }
        
        return newObject;
    }
    
    void CleanupUnused()
    {
        for (auto it = m_Cache.begin(); it != m_Cache.end();)
        {
            auto& object = it->second;
            
            if (object.use_count() <= 1)
            {
                it = m_Cache.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
    
    void Clear()
    {
        m_Cache.clear();
    }
    
    size_t Size() const
    {
        return m_Cache.size();
    }

private:
    ObjectCache() = default;
    ~ObjectCache() { Clear(); }
    
    ObjectCache(const ObjectCache&) = delete;
    ObjectCache& operator=(const ObjectCache&) = delete;
    Creator m_Creator;
    std::unordered_map<KeyT, PhysicsPtr<T>, Hasher> m_Cache;
};