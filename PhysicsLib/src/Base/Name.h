#pragma once
#include <string>
namespace PhysicsBase
{
    template<typename T>
    class Name
    {
    public:
        Name(T* object) : m_Object(object) {}
#ifdef PHYSX_IMPL
        const std::string& GetName() const { return m_Object->getName(); }
        void SetName(const std::string& name) { m_Object->setName(name.c_str()); }
    private:
#else
        const std::string& GetName() const { return m_Name; }
        void SetName(const std::string& name) { m_Name = name; }
#endif

    private:
        T* m_Object = nullptr;
#ifndef PHYSX_IMPL
        std::string m_Name;
#endif
    };
}
