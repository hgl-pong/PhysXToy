#pragma once
inline Magnum::Matrix4 ToMagnum(const MathLib::HMatrix4& mat) {
	Magnum::Matrix4 matrix;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix[i][j] = mat(i, j);
		}
	}
	return matrix;
}
inline MathLib::HMatrix4 FromMagnum(const Magnum::Matrix4& mat) {
	MathLib::HMatrix4 matrix;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix(i, j) = mat[i][j];
		}
	}
	return matrix;
}
inline Magnum::Vector3 ToMagnum(const MathLib::HVector3& vec) {
	return Magnum::Vector3(vec[0], vec[1], vec[2]);
}
inline MathLib::HVector3 FromMagnum(const Magnum::Vector3& vec) {
	return MathLib::HVector3(vec[0], vec[1], vec[2]);
}

inline MathLib::HQuaternion FromMagnum(const Magnum::Quaternion& quat) {
	return MathLib::HQuaternion(quat.scalar(), quat.vector()[0], quat.vector()[1], quat.vector()[2]);
}

inline Magnum::Quaternion ToMagnum(const MathLib::HQuaternion& quat) {
	return Magnum::Quaternion({ quat.x(), quat.y(), quat.z() }, quat.w());
}

//inline Trade::MeshData ToMagnum(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices) 
//{
//	Trade::MeshData meshData;
//	meshData.vertexCount = vertices.size();
//	meshData.indexCount = indices.size();
//	meshData.vertexData = new float[meshData.vertexCount * 3];
//	meshData.indexData = new uint32_t[meshData.indexCount];
//	for (size_t i = 0; i < vertices.size(); i++) {
//		meshData.vertexData[i * 3] = vertices[i][0];
//		meshData.vertexData[i * 3 + 1] = vertices[i][1];
//		meshData.vertexData[i * 3 + 2] = vertices[i][2];
//	}
//	for (size_t i = 0; i < indices.size(); i++) {
//		meshData.indexData[i] = indices[i];
//	}
//	return meshData;
//}