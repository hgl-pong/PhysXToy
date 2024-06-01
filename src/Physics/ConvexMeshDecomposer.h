#pragma once
#include "Physics/PhysicsCommon.h"
#include "VHACD/VHACD.h"
#include "OCLAcceleration.h"
#define PRINT_OCL_INFO 0

class ConvexMeshDecomposer
{
public:
	ConvexMeshDecomposer()
	{
		m_bUseOCLAcceleration = true;
		_InitOCLAcceleration();
	}

	~ConvexMeshDecomposer()
	{
		m_OCLAcceleration.reset();
	};

	void EnableOCLAcceleration(bool bEnable)
	{
		m_bUseOCLAcceleration = bEnable;
		_InitOCLAcceleration();
	}

	bool IsOCLAccelerationEnabled() const
	{
		return m_bUseOCLAcceleration;
	}

	bool Decompose(const PhysicsMeshData& meshData, const ConvexDecomposeOptions& params, std::vector<PhysicsMeshData>& convexMeshesData)
	{
		VHACD::IVHACD* m_VHACD = VHACD::CreateVHACD();

		VHACD::IVHACD::Parameters vhacdParams;
		vhacdParams.m_maxNumVerticesPerCH = params.m_MaximumNumberOfVerticesPerHull;
		vhacdParams.m_maxConvexHulls = params.m_MaximumNumberOfHulls;
		vhacdParams.m_resolution = params.m_VoxelGridResolution;
		vhacdParams.m_concavity = params.m_Concavity;
		vhacdParams.m_oclAcceleration = m_bUseOCLAcceleration;
		vhacdParams.m_minVolumePerCH = 0.003f;

		if (m_bUseOCLAcceleration)
		{
			_InitOCLAcceleration();
			bool res = m_VHACD->OCLInit(m_OCLAcceleration->GetDevice());
			if (!res)
			{
				vhacdParams.m_oclAcceleration = false;
			}
		}

		std::vector<float > vertices;
		vertices.resize(meshData.m_Vertices.size() * 3);

		MathLib::HAABBox3D aabb;
		aabb.setEmpty();
		for (size_t i = 0; i < meshData.m_Vertices.size(); i++)
		{
			aabb.extend(meshData.m_Vertices[i]);
		}

		MathLib::HVector3 min = aabb.min();
		MathLib::HVector3 max = aabb.max();

		for (size_t i = 0; i < meshData.m_Vertices.size(); i++)
		{
			const MathLib::HVector3& v = meshData.m_Vertices[i];
			vertices[i * 3] = v[0];
			vertices[i * 3 + 1] = v[1];
			vertices[i * 3 + 2] = v[2];
		}
		
		m_VHACD->Compute(vertices.data(), vertices.size()/3, meshData.m_Indices.data(), meshData.m_Indices.size() / 3, vhacdParams);

		const size_t nConvexHulls = m_VHACD->GetNConvexHulls();
		convexMeshesData.resize(nConvexHulls);

		for (size_t i = 0; i < nConvexHulls; i++)
		{
			VHACD::IVHACD::ConvexHull ch;
			m_VHACD->GetConvexHull(i, ch);
			PhysicsMeshData& convexMeshData = convexMeshesData[i];
			convexMeshData.m_Vertices.resize(ch.m_nPoints);
			for (size_t j = 0; j < ch.m_nPoints; j++)
			{
				convexMeshData.m_Vertices[j] = MathLib::HVector3(ch.m_points[j * 3], ch.m_points[j * 3 + 1], ch.m_points[j * 3 + 2]);
			}
			convexMeshData.m_Indices.resize(ch.m_nTriangles * 3);
			for (size_t j = 0; j < ch.m_nTriangles; j++)
			{
				convexMeshData.m_Indices[j * 3] = ch.m_triangles[j * 3];
				convexMeshData.m_Indices[j * 3 + 1] = ch.m_triangles[j * 3 + 1];
				convexMeshData.m_Indices[j * 3 + 2] = ch.m_triangles[j * 3 + 2];
			}
		}

		if (m_bUseOCLAcceleration)
		{			
			m_VHACD->OCLRelease();
		}
		if (m_VHACD)
		{
			m_VHACD->Release();
			m_VHACD = nullptr;
		}
		return true;
	}
private:
	void _InitOCLAcceleration()
	{
		if (m_bUseOCLAcceleration)
		{
			if (!m_OCLAcceleration)
			{
				m_OCLAcceleration = std::make_unique<OCLAcceleration>();
				bool res = InitOCL(0,
					0,
					*m_OCLAcceleration,PRINT_OCL_INFO);
				if (!res) {
					return ;
				}
			}
		}
		else
		{
			m_OCLAcceleration.reset();
		}
	}
private:
	bool m_bUseOCLAcceleration = false;
	std::unique_ptr<	OCLAcceleration> m_OCLAcceleration;

};