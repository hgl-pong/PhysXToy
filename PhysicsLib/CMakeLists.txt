cmake_minimum_required(VERSION 3.6)
project(PhysicsLib)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../executable/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../executable/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../executable/${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

file(GLOB PHYSICS_SOURCE_FILES
    src/VHACD/*.cpp
    src/Utility/*.cpp
    src/Base/*.cpp
    src/*.cpp
)

add_library(PhysicsLib SHARED ${PHYSICS_SOURCE_FILES})

add_definitions(-DOPENCL_FOUND)
add_definitions(-DOPENCL_CL_FILES=\"${PROJECT_SOURCE_DIR}/src/VHACD/cl/vhacdKernels.cl\")
add_definitions(-DUSE_OPENMP)
add_definitions(-DPHYSICSLIB_EXPORTS)

find_package(unofficial-omniverse-physx-sdk CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCL REQUIRED)

target_include_directories(PhysicsLib
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/ThirdParty/HMath/include
)

target_link_libraries(PhysicsLib
    PRIVATE
    unofficial::omniverse-physx-sdk::sdk
    Eigen3::Eigen
    OpenCL::OpenCL
)

add_custom_command(TARGET PhysicsLib POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-library>
    $<TARGET_FILE_DIR:PhysicsLib>)

add_custom_command(TARGET PhysicsLib POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-device-library>
    $<TARGET_FILE_DIR:PhysicsLib>)