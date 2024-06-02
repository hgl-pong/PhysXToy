cmake_minimum_required(VERSION 3.6)
set(PROJECT_NAME Toy)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)
# 全局设置使用动态运行时库
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

set(PHYSX_TOY PhysXToy)

file(GLOB SOURCE_FILES
    src/*
    src/Render/*
    src/Physics/*
    src/Physics/VHACD/*
    src/Physics/Utility/*
)

add_definitions(-DOPENCL_FOUND)
add_definitions(-DOPENCL_CL_FILES=\"${PROJECT_SOURCE_DIR}/src/Physics/VHACD/cl/vhacdKernels.cl\")
add_definitions(-DUSE_OPENMP)
add_executable(${PHYSX_TOY} ${SOURCE_FILES})
find_package(unofficial-omniverse-physx-sdk CONFIG REQUIRED)
find_package(GLEW REQUIRED)  
find_package(GLUT REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCL REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)
find_package(Magnum REQUIRED
    GL
    MeshTools
    Primitives
    SceneGraph
    Shaders
    Trade
    Text
    Sdl2Application)

target_include_directories(${PHYSX_TOY} PUBLIC ./include
    PRIVATE)

target_link_libraries(${PHYSX_TOY} PRIVATE 
    unofficial::omniverse-physx-sdk::sdk 
    GLEW::GLEW
    GLUT::GLUT
    Eigen3::Eigen 
    OpenCL::OpenCL 
    tinyobjloader::tinyobjloader
    Magnum::Application
    Magnum::GL
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::Shaders
    Magnum::Trade
    Magnum::Text
    Magnum::SceneGraph
    Magnum::Sdl2Application)

add_custom_command(TARGET ${PHYSX_TOY} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-library>
                        $<TARGET_FILE_DIR:${PHYSX_TOY}>)

add_custom_command(TARGET ${PHYSX_TOY} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-device-library>
                $<TARGET_FILE_DIR:${PHYSX_TOY}>)
