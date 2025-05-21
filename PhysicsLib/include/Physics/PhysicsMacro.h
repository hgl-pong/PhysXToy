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

#define PHYSICS_REPORT_ERROR(message, file, line) 

#define PHYSICS_STRINGIZE_HELPER(X) #X
#define PHYSICS_STRINGIZE(X) PHYSICS_STRINGIZE_HELPER(X)
#define PHYSICS_CONCAT_HELPER(X, Y) X##Y
#define PHYSICS_CONCAT(X, Y) PHYSICS_CONCAT_HELPER(X, Y)

#define PHYSICS_PRINT(msg, ...) printf(msg, __VA_ARGS__);

#define ASSERT_MSG(condition, message) if (!(condition)) { PHYSICS_REPORT_ERROR(message, __FILE__, __LINE__); }
