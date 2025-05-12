#pragma once

#include "PhysicsObjectPool.h"
#include <unordered_map>
#include <typeindex>
#include <memory>
#include <string>

class PhysicsRigidDynamic;
class PhysicsRigidStatic;
class PhysicsJoint;
class PhysicsCloth;
class PhysicsSoftBody;

class PhysicsObjectPoolsManager
{
public:
    static PhysicsObjectPoolsManager& GetInstance()
    {
        static PhysicsObjectPoolsManager instance;
        return instance;
    }
    
    class IPoolBase
    {
    public:
        virtual ~IPoolBase() = default;
        virtual void ClearInterface() = 0;
        virtual size_t GetSizeInterface() const = 0;
        virtual uint32_t GetMaxSizeInterface() const = 0;
    };
    
    template<typename T>
    class TypedPoolWrapper : public IPoolBase, public PhysicsObjectPool<T>
    {
    public:
        TypedPoolWrapper(uint32_t size) : PhysicsObjectPool<T>(size) {}
        
        void ClearInterface() override
        {
            this->Clear();
        }
        
        size_t GetSizeInterface() const override
        {
            return this->GetSize();
        }
        
        uint32_t GetMaxSizeInterface() const override
        {
            return this->GetMaxSize();
        }
    };
    
    template<typename T>
    PhysicsObjectPool<T>* GetPool()
    {
        std::type_index typeId = std::type_index(typeid(T));
        
        auto it = m_TypedPools.find(typeId);
        if (it != m_TypedPools.end())
        {
            return static_cast<TypedPoolWrapper<T>*>(it->second.get());
        }
        
        auto pool = std::make_unique<TypedPoolWrapper<T>>(GetDefaultPoolSize<T>());
        TypedPoolWrapper<T>* poolPtr = pool.get();
        m_TypedPools[typeId] = std::move(pool);
        return poolPtr;
    }
    
    template<typename T>
    T* TakeObject()
    {
        return GetPool<T>()->TakeObject();
    }
    
    template<typename T>
    void ReturnObject(T** object)
    {
        GetPool<T>()->ReturnObject(object);
    }
    
    template<typename T>
    size_t GetPoolSize()
    {
        return GetPool<T>()->GetSize();
    }
    
    template<typename T>
    uint32_t GetPoolMaxSize()
    {
        return GetPool<T>()->GetMaxSize();
    }
    
    template<typename T>
    void ResizePool(uint32_t newSize)
    {
        GetPool<T>()->Resize(newSize);
    }
    
    template<typename T>
    void ClearPool()
    {
        GetPool<T>()->Clear();
    }
    
    void ClearAllPools()
    {
        for (auto& pair : m_TypedPools)
        {
            pair.second->ClearInterface();
        }
    }
    
    template<typename T>
    void SetDefaultPoolSize(uint32_t size)
    {
        std::type_index typeId = std::type_index(typeid(T));
        m_DefaultPoolSizes[typeId] = size;
    }
    
    template<typename T>
    uint32_t GetDefaultPoolSize()
    {
        std::type_index typeId = std::type_index(typeid(T));
        auto it = m_DefaultPoolSizes.find(typeId);
        if (it != m_DefaultPoolSizes.end())
        {
            return it->second;
        }
        
        if (std::is_same<T, PhysicsRigidDynamic>::value) return 500;
        if (std::is_same<T, PhysicsRigidStatic>::value) return 500;
        if (std::is_same<T, PhysicsJoint>::value) return 200;
        if (std::is_same<T, PhysicsCloth>::value) return 50;
        if (std::is_same<T, PhysicsSoftBody>::value) return 50;
        
        return 100;
    }
    
private:
    PhysicsObjectPoolsManager() = default;
    ~PhysicsObjectPoolsManager() = default;
    
    PhysicsObjectPoolsManager(const PhysicsObjectPoolsManager&) = delete;
    PhysicsObjectPoolsManager& operator=(const PhysicsObjectPoolsManager&) = delete;
    
    std::unordered_map<std::type_index, std::unique_ptr<IPoolBase>> m_TypedPools;
    
    std::unordered_map<std::type_index, uint32_t> m_DefaultPoolSizes;
};

inline PhysicsObjectPoolsManager& GetPhysicsPoolsManager()
{
    return PhysicsObjectPoolsManager::GetInstance();
}

#define TAKE_PHYSICS_OBJECT(Type) GetPhysicsPoolsManager().TakeObject<Type>()
#define RETURN_PHYSICS_OBJECT(Type, Obj) GetPhysicsPoolsManager().ReturnObject<Type>(&Obj)
