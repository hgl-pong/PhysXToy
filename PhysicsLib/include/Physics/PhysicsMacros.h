#pragma once

#ifdef _WIN32
#ifdef PHYSICSLIB_EXPORTS
#define PHYSICSLIB_API __declspec(dllexport)
#else
#define PHYSICSLIB_API __declspec(dllimport)
#endif
#else
#define PHYSICSLIB_API
#endif 

#define PHYSICS_STRINGIZE_HELPER(X) #X
#define PHYSICS_STRINGIZE(X) PHYSICS_STRINGIZE_HELPER(X)
#define PHYSICS_CONCAT_HELPER(X, Y) X##Y
#define PHYSICS_CONCAT(X, Y) PHYSICS_CONCAT_HELPER(X, Y)

#ifdef _DEBUG
#define PHYSICS_DEBUG 1
#else
#define PHYSICS_DEBUG 0
#endif

#define PHYSICS_PRINT(msg, ...) printf(msg, ##__VA_ARGS__);
#define PHYSICS_LOG(msg, ...) printf("[Physics Log] " msg "\n", ##__VA_ARGS__)
#define PHYSICS_LOG_INFO(msg, ...) printf("[Info] " msg "\n", ##__VA_ARGS__)
#define PHYSICS_LOG_WARNING(msg, ...) printf("[Warning] " msg "\n", ##__VA_ARGS__)
#define PHYSICS_LOG_ERROR(msg, ...) printf("[Error] " msg "\n", ##__VA_ARGS__)

#define PHYSICS_REPORT_ERROR(message) \
    printf("ERROR [File:%s] [Line:%d]: %s\n", __FILE__, __LINE__, message);

#define PHYSICS_ASSERT(condition) \
    if(!(condition)) { PHYSICS_PRINT("Assertion failed: " #condition, __FILE__, __LINE__); }

#define PHYSICS_ASSERT_MSG(condition, message) \
    if(!(condition)) { PHYSICS_PRINT(message, __FILE__, __LINE__); }

#if PHYSICS_DEBUG
#define PHYSICS_DEBUG_ONLY(x) x
#else
#define PHYSICS_DEBUG_ONLY(x)
#endif

#define PHYSICS_SAFE_DELETE(p) if(p) { delete p; p = nullptr; }
#define PHYSICS_SAFE_DELETE_ARRAY(p) if(p) { delete[] p; p = nullptr; }
#define PHYSICS_SAFE_RELEASE(p) if(p) { p->Release(); p = nullptr; }

#define PHYSICS_PI 3.14159265358979323846f
#define PHYSICS_DEG_TO_RAD(deg) ((deg) * (PHYSICS_PI / 180.0f))
#define PHYSICS_RAD_TO_DEG(rad) ((rad) * (180.0f / PHYSICS_PI))

#define PHYSICS_METERS_TO_CENTIMETERS(m) ((m) * 100.0f)
#define PHYSICS_CENTIMETERS_TO_METERS(cm) ((cm) * 0.01f)

#define PHYSICS_DEPRECATED __declspec(deprecated)
#define PHYSICS_INLINE inline
#define PHYSICS_FORCE_INLINE __forceinline

#define PHYSICS_MAX_CONTACTS 8
#define PHYSICS_MAX_ITERATIONS 10
#define PHYSICS_MAX_BODIES 1024
#define PHYSICS_MAX_MANIFOLDS 1024

#define PHYSICS_EPSILON 1e-6f
#define PHYSICS_EQUALS(a, b) (fabsf((a) - (b)) <= PHYSICS_EPSILON)

#define ASSERT_MSG(condition, message) PHYSICS_ASSERT_MSG(condition, message)

#define PHYSICS_ENUM_OPERATOR(enum_type) \
    inline enum_type operator|(enum_type a, enum_type b) { return static_cast<enum_type>(static_cast<int>(a) | static_cast<int>(b)); } \
    inline enum_type operator&(enum_type a, enum_type b) { return static_cast<enum_type>(static_cast<int>(a) & static_cast<int>(b)); } \
    inline enum_type operator^(enum_type a, enum_type b) { return static_cast<enum_type>(static_cast<int>(a) ^ static_cast<int>(b)); } \
    inline enum_type operator~(enum_type a) { return static_cast<enum_type>(~static_cast<int>(a)); } \
    inline enum_type& operator|=(enum_type& a, enum_type b) { a = a | b; return a; } \
    inline enum_type& operator&=(enum_type& a, enum_type b) { a = a & b; return a; } \
    inline enum_type& operator^=(enum_type& a, enum_type b) { a = a ^ b; return a; }
