#pragma once
#include "tiny_obj_loader.h"
inline bool LoadObj(const char *filename, PhysicsMeshData &meshdata ,const MathLib::HReal& scale)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    if (!tinyobj::LoadObj(&attrib, &shapes, nullptr, nullptr, nullptr, filename))
        return false;
    if(shapes.size()!=1)
		return false;

    const tinyobj::shape_t& shape = shapes[0];
    size_t index_offset = 0;

    for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
        int fv = shape.mesh.num_face_vertices[f];

        if (fv != 3) {
            return false;
        }

        for (size_t v = 0; v < fv; v++) {
            tinyobj::index_t idx = shape.mesh.indices[index_offset + v];
            tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0] * scale;
            tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1] * scale;
            tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2] * scale;

            MathLib::HVector3 vertex(vx, vy, vz);
            auto it = std::find(meshdata.m_Vertices.begin(), meshdata.m_Vertices.end(), vertex);
            if (it == meshdata.m_Vertices.end()) {
                meshdata.m_Vertices.push_back(vertex);
                meshdata.m_Indices.push_back(static_cast<uint32_t>(meshdata.m_Vertices.size() - 1));
            }
            else {
                meshdata.m_Indices.push_back(static_cast<uint32_t>(std::distance(meshdata.m_Vertices.begin(), it)));
            }
        }
        index_offset += fv;
    }

    return true;
}

inline PhysicsMeshData GenerateCapsuleMeshData(float radius, float halfHeight, const int slices=10,const int stacks=10) {
    PhysicsMeshData meshData;

    std::vector<MathLib::HVector3>& vertices = meshData.m_Vertices;
    std::vector<uint32_t>& indices = meshData.m_Indices;

    // Generate vertices
    for (int i = 0; i <= stacks; ++i) {
        float phi = MathLib::H_PI * float(i) / float(stacks);
        float cosPhi = cos(phi);
        float sinPhi = sin(phi);

        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * MathLib::H_PI * float(j) / float(slices);
            float cosTheta = cos(theta);
            float sinTheta = sin(theta);

            MathLib::HVector3 vertex;
            vertex[0] = radius * cosPhi + (i <= stacks / 2 ? halfHeight : -halfHeight);
            vertex[1] = radius * sinPhi * cosTheta;
            vertex[2] = radius * sinPhi * sinTheta;
            vertices.push_back(vertex);
        }
    }

    // Generate indices
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int first = i * (slices + 1) + j;
            int second = first + slices + 1;

            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(first + 1);

            indices.push_back(second);
            indices.push_back(second + 1);
            indices.push_back(first + 1);
        }
    }
    return meshData;
}