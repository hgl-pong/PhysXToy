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