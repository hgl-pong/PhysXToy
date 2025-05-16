cmake_minimum_required(VERSION 3.6)
set(PROJECT_NAME Physics)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/executable/${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

add_subdirectory(PhysicsLib)

set(PHYSICS_TEST PhysicsTest)

file(GLOB SOURCE_FILES
    src/*
    src/Render/*
    src/Test/*
)

add_executable(${PHYSICS_TEST} ${SOURCE_FILES})
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCL REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(Jolt CONFIG REQUIRED)

target_include_directories(${PHYSICS_TEST} PUBLIC ./include
    PRIVATE
    ThirdParty/HMath/include
    PhysicsLib/include
)

target_link_libraries(${PHYSICS_TEST} PRIVATE
    Eigen3::Eigen
    tinyobjloader::tinyobjloader
    glfw
    imgui::imgui
    Jolt::Jolt
    PhysicsLib
)

add_custom_command(TARGET ${PHYSICS_TEST} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/asset
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/asset
)
