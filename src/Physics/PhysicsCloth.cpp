#include "PhysicsCloth.h"
#include "PhysicsEngine.h"
#include "PxPhysicsAPI.h"
#include "PhysicsMaterial.h"
#include "Utility/PhysXUtils.h"

using namespace physx;

PhysicsCloth::PhysicsCloth(PhysicsPtr<IPhysicsMaterial>& material)
    : m_Material(material)
{
    m_Type = PhysicsObjectType::PHYSICS_OBJECT_TYPE_CLOTH;
}

PhysicsCloth::~PhysicsCloth()
{
    Release();
}

void PhysicsCloth::Release()
{
    if (m_ParticleSystem && m_ClothBuffer)
    {
        m_ParticleSystem->removeParticleBuffer(m_ClothBuffer.get());
    }
    m_ParticleSystem = nullptr;
    m_ClothBuffer = nullptr;
    m_TriangleMesh = nullptr;
    m_PBDMaterial = nullptr;
    m_ColliderGeometries.clear();
    m_ColliderLocalPos.clear();
    m_FixedVertices.clear();
    m_OriginalVertices.clear();
    m_DeformedVertices.clear();
    m_TriangleIndices.clear();
}

void PhysicsCloth::Update()
{
    if (!m_ParticleSystem || !m_ClothBuffer)
        return;
    UpdateDeformedVertices();
    UpdateBoundingBox();
}

bool PhysicsCloth::AddColliderGeometry(PhysicsPtr<IColliderGeometry>& colliderGeometry, const MathLib::HTransform3& localTrans)
{
    if (!m_ParticleSystem || !m_ClothBuffer || !colliderGeometry)
        return false;
    
    m_ColliderGeometries.push_back(colliderGeometry);
    m_ColliderLocalPos.push_back(localTrans);

    return true;
}

bool PhysicsCloth::RemoveColliderGeometry(PhysicsPtr<IColliderGeometry>& colliderGeometry)
{
    if (!m_ParticleSystem || !m_ClothBuffer || !colliderGeometry)
        return false;
    
    for (size_t i = 0; i < m_ColliderGeometries.size(); i++)
    {
        if (m_ColliderGeometries[i] == colliderGeometry)
        {
            m_ColliderGeometries.erase(m_ColliderGeometries.begin() + i);
            m_ColliderLocalPos.erase(m_ColliderLocalPos.begin() + i);
            
            return true;
        }
    }
    
    return false;
}

void PhysicsCloth::GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>>& geomeries, std::vector<MathLib::HTransform3>* geoLocalPos)
{
    geomeries = m_ColliderGeometries;
    if (geoLocalPos)
        *geoLocalPos = m_ColliderLocalPos;
}

size_t PhysicsCloth::GetOffset() const
{
    return reinterpret_cast<size_t>(m_ParticleSystem.get());
}

void PhysicsCloth::SetTransform(const MathLib::HTransform3& trans)
{
    m_Transform = trans;
}

MathLib::HAABBox3D PhysicsCloth::GetWorldBoundingBox() const
{
    if (!m_ParticleSystem || !m_ClothBuffer)
        return MathLib::HAABBox3D();
    
    MathLib::HAABBox3D worldBBox = m_BoundingBox;
    worldBBox.transform(m_Transform);
    return worldBBox;
}

void PhysicsCloth::SetParameter(const ClothParams& params)
{
    if (!m_ParticleSystem || !m_ClothBuffer || !m_PBDMaterial)
        return;
    
    m_Params = params;

    m_PBDMaterial->setFriction(params.m_Friction);
    
    m_ParticleSystem->setSolverIterationCounts(params.m_SolverIterations);
    
    if (params.m_WindStrength > 0)
    {
        PxVec3 windDir = ConvertUtils::ToPx(params.m_WindDirection);
        windDir.normalize();
    }
}

uint32_t PhysicsCloth::GetVertexCount() const
{
    return m_OriginalVertices.size();
}

uint32_t PhysicsCloth::GetTriangleCount() const
{
    return m_TriangleIndices.size() / 3;
}

void PhysicsCloth::GetDeformedVertexPositions(std::vector<MathLib::HVector3>& positions) const
{
    if (!m_ParticleSystem || !m_ClothBuffer || m_DeformedVertices.empty())
    {
        positions = m_OriginalVertices;
        return;
    }
    
    positions = m_DeformedVertices;
}

void PhysicsCloth::GetTriangleIndices(std::vector<uint32_t>& indices) const
{
    indices = m_TriangleIndices;
}

void PhysicsCloth::SetVertexFixed(uint32_t vertexIndex, bool fixed)
{
    if (vertexIndex >= m_FixedVertices.size())
        return;
    
    m_FixedVertices[vertexIndex] = fixed;
    
}

bool PhysicsCloth::IsVertexFixed(uint32_t vertexIndex) const
{
    if (vertexIndex >= m_FixedVertices.size())
        return false;
    
    return m_FixedVertices[vertexIndex];
}

void PhysicsCloth::ApplyForceToVertex(uint32_t vertexIndex, const MathLib::HVector3& force)
{
    if (!m_ParticleSystem || !m_ClothBuffer || vertexIndex >= m_OriginalVertices.size())
        return;

}

void PhysicsCloth::ApplyWindForce(const MathLib::HVector3& windDirection, MathLib::HReal strength)
{
    if (!m_ParticleSystem)
        return;
    
    m_Params.m_WindDirection = windDirection;
    m_Params.m_WindStrength = strength;
    
}

void PhysicsCloth::SetMass(MathLib::HReal mass)
{
    if (!m_ParticleSystem || !m_ClothBuffer)
        return;
    
    m_Params.m_Mass = mass;
}

void PhysicsCloth::GetOriginalVertexPositions(std::vector<MathLib::HVector3>& positions) const
{
    positions = m_OriginalVertices;
}

void* PhysicsCloth::GetClothData() const
{
    return m_ClothBuffer.get();
}

void PhysicsCloth::Commit()
{
    if (!m_ParticleSystem || !m_ClothBuffer)
        return;
    
    SetParameter(m_Params);
}

void PhysicsCloth::GetRenderMesh(std::vector<MathLib::HVector3>& vertices, std::vector<uint32_t>& indices) const
{
    GetDeformedVertexPositions(vertices);
    indices = m_TriangleIndices;
}

bool PhysicsCloth::CreateFromMesh(const ClothCreateOptions& options)
{
    m_Transform = options.m_Transform;
    m_Params = options.m_Params;
    m_UserData = options.m_UserData;
    m_CollisionLayer = options.m_CollisionLayer;
    m_CollisionMask = options.m_CollisionMask;
    
    if (options.m_MeshDesc.m_Width > 0 && options.m_MeshDesc.m_Height > 0)
    {
        m_Width = options.m_MeshDesc.m_Width;
        m_Height = options.m_MeshDesc.m_Height;
        CreateGridMesh(m_Width, m_Height, options.m_MeshDesc.m_ParticleSpacing);
    }
    else if (!options.m_MeshDesc.m_Vertices.empty())
    {
        if (!CreateClothMesh(options.m_MeshDesc))
            return false;
    }
    else
    {
        return false;
    }
    
    return InitializeCloth(options);
}

bool PhysicsCloth::InitializeCloth(const ClothCreateOptions& options)
{
    PhysicsEngine* engine = static_cast<PhysicsEngine*>(PhysicsEngineUtils::GetPhysicsEngine());
    if (!engine)
        return false;
    
    PxCudaContextManager* cudaContextManager = nullptr;
    
    if (engine->GetActiveScene())
    {
        physx::PxScene* pxScene = static_cast<physx::PxScene*>(engine->GetActiveScene()->GetNativeScene());
        if (pxScene)
        {
            cudaContextManager = pxScene->getCudaContextManager();
        }
    }
    
    if (!cudaContextManager)
    {
        return false;
    }
    
    PxPBDMaterial* pbdMaterial = PxGetPhysics().createPBDMaterial(
        options.m_Params.m_Friction, 
        0.05f,                      
        1e+6f,                      
        0.001f,                     
        0.5f,                       
        0.005f,                     
        0.05f,                      
        0.0f,                       
        0.0f                        
    );
    
    if (!pbdMaterial)
        return false;
    
    m_PBDMaterial = make_physx_ptr(pbdMaterial);
    
    PxPBDParticleSystem* particleSystem = PxGetPhysics().createPBDParticleSystem(*cudaContextManager);
    if (!particleSystem)
        return false;
    
    m_ParticleSystem = make_physx_ptr(particleSystem);
    
    const PxReal restOffset = options.m_MeshDesc.m_ParticleSpacing * 0.5f;
    m_ParticleSystem->setRestOffset(restOffset);
    m_ParticleSystem->setContactOffset(restOffset + 0.02f);
    m_ParticleSystem->setParticleContactOffset(restOffset + 0.02f);
    m_ParticleSystem->setSolidRestOffset(restOffset);
    m_ParticleSystem->setFluidRestOffset(0.0f);

    if (engine->GetActiveScene())
    {
        physx::PxScene* pxScene = static_cast<physx::PxScene*>(engine->GetActiveScene()->GetNativeScene());
        if (pxScene)
        {
            pxScene->addActor(*particleSystem);
        }
    }
    
    const PxU32 particlePhase = m_ParticleSystem->createPhase(
        pbdMaterial, 
        PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter | PxParticlePhaseFlag::eParticlePhaseSelfCollide)
    );
    
    const PxU32 numParticles = m_OriginalVertices.size();
    const PxU32 numTriangles = m_TriangleIndices.size() / 3;
    
    PxParticleBuffer* buffer = PxGetPhysics().createParticleBuffer(numParticles, numParticles, cudaContextManager);
    if (!buffer)
    {
        return false;
    }
    
    PxVec4* positions = buffer->getPositionInvMasses();
    PxVec4* velocities = buffer->getVelocities();
    PxU32* phases = buffer->getPhases();
    
    const PxReal particleMass = options.m_Params.m_Mass / numParticles;
    
    for (PxU32 i = 0; i < numParticles; i++)
    {
        phases[i] = particlePhase;
        
        positions[i].x = m_OriginalVertices[i][0] + m_Transform.translation()[0];
        positions[i].y = m_OriginalVertices[i][1] + m_Transform.translation()[1];
        positions[i].z = m_OriginalVertices[i][2] + m_Transform.translation()[2];
        positions[i].w = 1.0f / particleMass;
        
        if (m_FixedVertices[i])
        {
            positions[i].w = 0.0f;
        }
        
        velocities[i] = PxVec4(0.0f);
    }
    
    buffer->setNbActiveParticles(numParticles);
    
    m_ParticleSystem->addParticleBuffer(buffer);
    
    m_ClothBuffer = make_physx_ptr(static_cast<PxParticleClothBuffer*>(buffer));
    
    for (PxU32 i = 0; i < numTriangles; i++)
    {
        PxU32 idx0 = m_TriangleIndices[i * 3];
        PxU32 idx1 = m_TriangleIndices[i * 3 + 1];
        PxU32 idx2 = m_TriangleIndices[i * 3 + 2];
        
        PxVec3 pos0(positions[idx0].x, positions[idx0].y, positions[idx0].z);
        PxVec3 pos1(positions[idx1].x, positions[idx1].y, positions[idx1].z);
        PxVec3 pos2(positions[idx2].x, positions[idx2].y, positions[idx2].z);
        
        PxReal rest01 = (pos0 - pos1).magnitude();
        PxReal rest02 = (pos0 - pos2).magnitude();
        PxReal rest12 = (pos1 - pos2).magnitude();
        
        const PxReal stretchStiffness = options.m_Params.m_Stiffness * 10000.0f;
        const PxReal springDamping = options.m_Params.m_Damping * 0.001f;
        
        //m_ParticleSystem->addSpring(idx0, idx1, rest01, stretchStiffness, springDamping);
        //m_ParticleSystem->addSpring(idx0, idx2, rest02, stretchStiffness, springDamping);
        //m_ParticleSystem->addSpring(idx1, idx2, rest12, stretchStiffness, springDamping);
    }
    
    SetParameter(m_Params);
    UpdateDeformedVertices();
    UpdateBoundingBox();
    
    return true;
}

bool PhysicsCloth::CreateClothMesh(const ClothMeshDesc& meshDesc)
{
    if (meshDesc.m_Vertices.empty())
        return false;
    
    m_OriginalVertices = meshDesc.m_Vertices;
    m_TriangleIndices = meshDesc.m_Indices;
    
    m_FixedVertices.resize(m_OriginalVertices.size(), false);
    for (uint32_t idx : meshDesc.m_FixedVertices)
    {
        if (idx < m_FixedVertices.size())
            m_FixedVertices[idx] = true;
    }
    
    return true;
}

void PhysicsCloth::UpdateBoundingBox()
{
    std::vector<MathLib::HVector3> deformedPositions;
    GetDeformedVertexPositions(deformedPositions);
    
    if (deformedPositions.empty())
    {
        m_BoundingBox = MathLib::HAABBox3D();
        return;
    }
    
    m_BoundingBox = MathLib::HAABBox3D(deformedPositions[0], deformedPositions[0]);
    for (size_t i = 1; i < deformedPositions.size(); i++)
    {
        m_BoundingBox.extend(deformedPositions[i]);
    }
}

void PhysicsCloth::CreateGridMesh(int width, int height, MathLib::HReal spacing)
{
    if (width <= 1 || height <= 1)
        return;
    
    m_OriginalVertices.resize(width * height);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int index = y * width + x;
            MathLib::HReal xPos = x * spacing - (width - 1) * spacing * 0.5f;
            MathLib::HReal yPos = 0.0f;
            MathLib::HReal zPos = y * spacing - (height - 1) * spacing * 0.5f;
            m_OriginalVertices[index] = MathLib::HVector3(xPos, yPos, zPos);
        }
    }
    
    m_TriangleIndices.clear();
    for (int y = 0; y < height - 1; y++)
    {
        for (int x = 0; x < width - 1; x++)
        {
            int idx0 = y * width + x;
            int idx1 = y * width + (x + 1);
            int idx2 = (y + 1) * width + x;
            int idx3 = (y + 1) * width + (x + 1);
            
            m_TriangleIndices.push_back(idx0);
            m_TriangleIndices.push_back(idx1);
            m_TriangleIndices.push_back(idx2);
            
            m_TriangleIndices.push_back(idx1);
            m_TriangleIndices.push_back(idx3);
            m_TriangleIndices.push_back(idx2);
        }
    }
    
    m_FixedVertices.resize(m_OriginalVertices.size(), false);
    
    for (int x = 0; x < width; x++)
    {
        m_FixedVertices[x] = true;
    }
}

void PhysicsCloth::UpdateDeformedVertices()
{
    if (m_ParticleSystem == nullptr || m_ClothBuffer == nullptr)
        return;

    const uint32_t numVertices = GetVertexCount();
    m_DeformedVertices.resize(numVertices);

    PxVec4* positions = m_ClothBuffer->getPositionInvMasses();
    PxU32 numParticles = m_ClothBuffer->getNbActiveParticles();
    
    if (positions && numParticles > 0)
    {
        const uint32_t count = std::min(static_cast<uint32_t>(numParticles), numVertices);
        for (uint32_t i = 0; i < count; i++)
        {
            m_DeformedVertices[i][0] = positions[i].x;
            m_DeformedVertices[i][1] = positions[i].y;
            m_DeformedVertices[i][2] = positions[i].z;
        }
    }
    else
    {
        m_DeformedVertices = m_OriginalVertices;
    }
    
    UpdateBoundingBox();
} 