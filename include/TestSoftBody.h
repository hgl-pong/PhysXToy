#pragma once
#include "Physics/PhysicsCommon.h"
#include <random>

inline void CreateCubeTetrahedronMesh(SoftBodyMeshDesc& meshDesc, float size = 1.0f)
{
    meshDesc.m_Vertices.clear();
    meshDesc.m_Indices.clear();
    meshDesc.m_FixedVertices.clear();
    
    float halfSize = size * 0.5f;
    
    meshDesc.m_Vertices.push_back(MathLib::HVector3(-halfSize, -halfSize, -halfSize)); // 0
    meshDesc.m_Vertices.push_back(MathLib::HVector3( halfSize, -halfSize, -halfSize)); // 1
    meshDesc.m_Vertices.push_back(MathLib::HVector3( halfSize,  halfSize, -halfSize)); // 2
    meshDesc.m_Vertices.push_back(MathLib::HVector3(-halfSize,  halfSize, -halfSize)); // 3
    
    meshDesc.m_Vertices.push_back(MathLib::HVector3(-halfSize, -halfSize,  halfSize)); // 4
    meshDesc.m_Vertices.push_back(MathLib::HVector3( halfSize, -halfSize,  halfSize)); // 5
    meshDesc.m_Vertices.push_back(MathLib::HVector3( halfSize,  halfSize,  halfSize)); // 6
    meshDesc.m_Vertices.push_back(MathLib::HVector3(-halfSize,  halfSize,  halfSize)); // 7
    
    meshDesc.m_Vertices.push_back(MathLib::HVector3(0.0f, 0.0f, 0.0f)); // 8
    
    meshDesc.m_Indices.push_back(0); meshDesc.m_Indices.push_back(1); meshDesc.m_Indices.push_back(2); meshDesc.m_Indices.push_back(8);
    meshDesc.m_Indices.push_back(0); meshDesc.m_Indices.push_back(1); meshDesc.m_Indices.push_back(2); meshDesc.m_Indices.push_back(8);
    meshDesc.m_Indices.push_back(0); meshDesc.m_Indices.push_back(2); meshDesc.m_Indices.push_back(3); meshDesc.m_Indices.push_back(8);
    
    meshDesc.m_Indices.push_back(1); meshDesc.m_Indices.push_back(5); meshDesc.m_Indices.push_back(6); meshDesc.m_Indices.push_back(8);
    meshDesc.m_Indices.push_back(1); meshDesc.m_Indices.push_back(6); meshDesc.m_Indices.push_back(2); meshDesc.m_Indices.push_back(8);
    
    meshDesc.m_Indices.push_back(5); meshDesc.m_Indices.push_back(4); meshDesc.m_Indices.push_back(7); meshDesc.m_Indices.push_back(8);
    meshDesc.m_Indices.push_back(5); meshDesc.m_Indices.push_back(7); meshDesc.m_Indices.push_back(6); meshDesc.m_Indices.push_back(8);
    
    meshDesc.m_Indices.push_back(4); meshDesc.m_Indices.push_back(0); meshDesc.m_Indices.push_back(3); meshDesc.m_Indices.push_back(8);
    meshDesc.m_Indices.push_back(4); meshDesc.m_Indices.push_back(3); meshDesc.m_Indices.push_back(7); meshDesc.m_Indices.push_back(8);
    
    meshDesc.m_Indices.push_back(3); meshDesc.m_Indices.push_back(2); meshDesc.m_Indices.push_back(6); meshDesc.m_Indices.push_back(8);
    meshDesc.m_Indices.push_back(3); meshDesc.m_Indices.push_back(6); meshDesc.m_Indices.push_back(7); meshDesc.m_Indices.push_back(8);
    
    meshDesc.m_Indices.push_back(1); meshDesc.m_Indices.push_back(0); meshDesc.m_Indices.push_back(4); meshDesc.m_Indices.push_back(8);
    meshDesc.m_Indices.push_back(1); meshDesc.m_Indices.push_back(4); meshDesc.m_Indices.push_back(5); meshDesc.m_Indices.push_back(8);
    
    meshDesc.m_FixedVertices.push_back(2); 
    meshDesc.m_FixedVertices.push_back(6); 
}

inline void CreateSphereTetrahedronMesh(SoftBodyMeshDesc& meshDesc, float radius = 1.0f, int resolution = 10)
{
    meshDesc.m_Vertices.clear();
    meshDesc.m_Indices.clear();
    meshDesc.m_FixedVertices.clear();
    
    meshDesc.m_Vertices.push_back(MathLib::HVector3(0.0f, 0.0f, 0.0f)); // 0
    
    std::vector<MathLib::HVector3> spherePoints;
    
    float goldenRatio = (1.0f + std::sqrt(5.0f)) / 2.0f;
    int numPoints = resolution * resolution;
    
    for (int i = 0; i < numPoints; i++)
    {
        float t = (float)i / numPoints;
        float inclination = std::acos(1.0f - 2.0f * t);
        float azimuth = 2.0f * MathLib::H_PI * goldenRatio * i;
        
        float x = radius * std::sin(inclination) * std::cos(azimuth);
        float y = radius * std::sin(inclination) * std::sin(azimuth);
        float z = radius * std::cos(inclination);
        
        spherePoints.push_back(MathLib::HVector3(x, y, z));
    }
    
    for (const auto& point : spherePoints)
    {
        meshDesc.m_Vertices.push_back(point);
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(1, spherePoints.size() - 1);
    
    for (int i = 0; i < numPoints * 2; i++)
    {
        int idx1 = distrib(gen);
        int idx2 = distrib(gen);
        int idx3 = distrib(gen);
        
        while (idx2 == idx1) idx2 = distrib(gen);
        while (idx3 == idx1 || idx3 == idx2) idx3 = distrib(gen);
        
        meshDesc.m_Indices.push_back(0);  
        meshDesc.m_Indices.push_back(idx1);
        meshDesc.m_Indices.push_back(idx2);
        meshDesc.m_Indices.push_back(idx3);
    }
    
    for (int i = 0; i < 3; i++)
    {
        meshDesc.m_FixedVertices.push_back(distrib(gen));
    }
}

inline void CreateSoftBodyTestScene()
{
    PhysicsEngineOptions engineOptions;
    engineOptions.m_bEnablePVD = true;
    engineOptions.m_EnableDebugVisualization = true;
    
    IPhysicsEngine* physicsEngine = PhysicsEngineUtils::CreatePhysicsEngine(engineOptions);
    
    PhysicsSceneCreateOptions sceneOptions;
    sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);
    sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
    
    PhysicsPtr<IPhysicsScene> scene = physicsEngine->CreateScene(sceneOptions);
    physicsEngine->SetActiveScene(scene);
    
    PhysicsObjectCreateOptions groundOptions;
    groundOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
    groundOptions.m_Transform.setIdentity();
    groundOptions.m_Transform.translate(MathLib::HVector3(0.0f, -5.0f, 0.0f));
    
    PhysicsPtr<IPhysicsObject> ground = physicsEngine->CreateObject(groundOptions);
    
    CollisionGeometryCreateOptions groundGeomOptions;
    groundGeomOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
    groundGeomOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(20.0f, 1.0f, 20.0f);
    
    PhysicsPtr<IColliderGeometry> groundGeometry = physicsEngine->CreateColliderGeometry(groundGeomOptions);
    ground->AddColliderGeometry(groundGeometry, MathLib::HTransform3());
    
    scene->AddPhysicsObject(ground);
    
    SoftBodyCreateOptions softBodyOptions;
    
    softBodyOptions.m_MaterialOptions.m_StaticFriction = 0.5f;
    softBodyOptions.m_MaterialOptions.m_DynamicFriction = 0.5f;
    softBodyOptions.m_MaterialOptions.m_Restitution = 0.2f;
    
    softBodyOptions.m_Params.m_YoungModulus = 1e+6f;  
    softBodyOptions.m_Params.m_PoissonRatio = 0.45f;  
    softBodyOptions.m_Params.m_Damping = 0.1f;        
    softBodyOptions.m_Params.m_Mass = 10.0f;          
    softBodyOptions.m_Params.m_SolverIterations = 10; 
    
    softBodyOptions.m_Transform.setIdentity();
    softBodyOptions.m_Transform.translate(MathLib::HVector3(0.0f, 5.0f, 0.0f));
    
    bool createCube = true;
    if (createCube)
    {
        CreateCubeTetrahedronMesh(softBodyOptions.m_MeshDesc, 2.0f);
    }
    else
    {
        CreateSphereTetrahedronMesh(softBodyOptions.m_MeshDesc, 1.5f, 10);
    }
    
    PhysicsPtr<ISoftBody> softBody = physicsEngine->CreateSoftBody(softBodyOptions);
    
    if (softBody)
    {
        scene->AddPhysicsObject(softBody);
        
        softBody->SetVertexFixed(0, true); 
        
        softBody->Commit();
    }
} 