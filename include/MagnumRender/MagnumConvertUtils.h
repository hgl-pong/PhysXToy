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

inline Magnum::Matrix3 ToMagnum(const MathLib::HMatrix3& mat) {
	Magnum::Matrix3 matrix;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			matrix[i][j] = mat(i, j);
		}
	}
	return matrix;
}

inline MathLib::HMatrix3 FromMagnum(const Magnum::Matrix3& mat) {
	MathLib::HMatrix3 matrix;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			matrix(i, j) = mat[i][j];
		}
	}
	return matrix;
}

inline Magnum::Trade::MeshData CreateMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices)
{
	std::vector<MathLib::HVector3> normals0(vertices.size(), MathLib::HVector3(0, 0, 0));
	for (size_t i = 0; i < indices.size(); i += 3)
	{
		const MathLib::HVector3& v0 = vertices[indices[i]];
		const MathLib::HVector3& v1 = vertices[indices[i + 1]];
		const MathLib::HVector3& v2 = vertices[indices[i + 2]];
		MathLib::HVector3 normal = (v1 - v0).cross(v2 - v0);
		normals0[indices[i]] += normal;
		normals0[indices[i + 1]] += normal;
		normals0[indices[i + 2]] += normal;
	}
	for (auto& normal : normals0)
		normal.normalize();

	size_t vertexCount = vertices.size();
	Corrade::Containers::Array<char> vertexData{ Corrade::Containers::NoInit, vertexCount * (sizeof(Magnum::Vector3) + sizeof(Magnum::Vector3)) };
	auto positions = Corrade::Containers::arrayCast<Magnum::Vector3>(vertexData.prefix(vertexCount * sizeof(Magnum::Vector3)));
	auto normals = Corrade::Containers::arrayCast<Magnum::Vector3>(vertexData.suffix(vertexCount * sizeof(Magnum::Vector3)));

	for (size_t i = 0; i < vertexCount; ++i)
	{
		new (&positions[i]) Magnum::Vector3(ToMagnum(vertices[i]));
		new (&normals[i]) Magnum::Vector3(ToMagnum(normals0[i]));
	}

	Corrade::Containers::Array<char> indexData{ Corrade::Containers::NoInit, indices.size() * sizeof(uint32_t) };
	auto indicesArray = Corrade::Containers::arrayCast<uint32_t>(indexData);

	for (size_t i = 0; i < indices.size(); ++i)
		new (&indicesArray[i]) uint32_t(indices[i]);

	return Magnum::Trade::MeshData{
		Magnum::MeshPrimitive::Lines,
		std::move(indexData),
		Magnum::Trade::MeshIndexData{indicesArray},
		std::move(vertexData),
		{Magnum::Trade::MeshAttributeData{Magnum::Trade::MeshAttribute::Position, positions},
			Magnum::Trade::MeshAttributeData{Magnum::Trade::MeshAttribute::Normal, normals}} };
}