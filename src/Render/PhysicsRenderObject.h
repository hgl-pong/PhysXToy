#pragma once
#include <Magnum/Magnum.h>
#include <Magnum/Image.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Grid.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/VertexColor.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/SceneGraph/Drawable.h>
#include "Physics/PhysicsCommon.h"
#include "MeshDataLoader.h"
namespace Magnum
{
	static MathLib::HVector3 mLightPosition(7.0f, 5.0f, 2.5f);
	static Magnum::Color4 mLightColor(0.5f, 0.5f, 0.5f, 1.0f);
	using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
	using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;
	using namespace Math::Literals;

	class FlatDrawable : public SceneGraph::Drawable3D {
	public:
		explicit FlatDrawable(Object3D& object, Shaders::Flat3D& shader, const Trade::MeshData& meshData, SceneGraph::DrawableGroup3D& drawables) :
			SceneGraph::Drawable3D{ object, &drawables }, m_FlatShader(shader), m_Mesh(MeshTools::compile(meshData)) {}

		void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
			m_FlatShader
				.setColor(m_Color)
				.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
				.draw(m_Mesh);
		}

		void Show(bool show) {
			m_bShow = show;
		}

		void SetColor(const Color4& color) {
			m_Color = color;
		}

		void SetMeshData(Trade::MeshData& meshdata)
		{
			m_Mesh = MeshTools::compile(meshdata);
		}

	private:
		bool m_bShow = true;
		Shaders::Flat3D& m_FlatShader;
		GL::Mesh m_Mesh;
		Color4 m_Color = 0x747474_rgbf;
	};

	class RenderableObject : public SceneGraph::Drawable3D {
	public:
		explicit RenderableObject(Object3D* object,Shaders::Phong& pshader, Shaders::Flat3D&fShader, const Trade::MeshData& meshData, SceneGraph::DrawableGroup3D& group, const Matrix4& matrix) :
			SceneGraph::Drawable3D{ *object, &group }, m_PhongShader(pshader),m_FlatShader(fShader),m_Mesh(MeshTools::compile(meshData)), m_Object(object ) , m_LocalPos(matrix){}

		void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override {
			if (!m_bShow)
				return;
			m_Mesh.setPrimitive(MeshPrimitive::Triangles);
			m_PhongShader.setLightPosition(ToMagnum(mLightPosition))
				.setAmbientColor(IsSleeping? m_AmbientColor*0.5:m_AmbientColor)
				.setDiffuseColor(m_DiffuseColor)
				.setLightColor(mLightColor)
				.setTransformationMatrix(transformationMatrix)
				.setNormalMatrix(transformationMatrix.normalMatrix())
				.setProjectionMatrix(camera.projectionMatrix());
			m_Mesh.draw(m_PhongShader);	
			if (m_bShowWireframe)
			{
				m_Mesh.setPrimitive(MeshPrimitive::Lines);
				m_FlatShader.setColor(0x000000_rgbf)
					.setTransformationProjectionMatrix(camera.projectionMatrix() * transformationMatrix)
					.draw(m_Mesh);
			}
		}

		void SetAmbientColor(const Color4& color) {
			m_AmbientColor = color;
		}
		void SetDiffuseColor(const Color4& color) {
			m_DiffuseColor = color;
		}

		void SetIsSleeping(bool isSleeping)
		{
			IsSleeping = isSleeping;
		}

		void ShowWireframe(bool show) {
			m_bShowWireframe = show;
		}

		void SetMeshData(Trade::MeshData& meshdata)
		{
			m_Mesh = MeshTools::compile(meshdata);
		}

		void Show(bool show)
		{
			m_bShow = show;
		}
	private:
		bool m_bShow = true;
		bool IsSleeping = false;
		bool m_bShowWireframe = true;
		Color4 m_AmbientColor = { 1.f,1.f,1.f,1.f };
		Color4 m_DiffuseColor = { 1.f,1.f,1.f,1.f };

		Shaders::Phong& m_PhongShader;
		Shaders::Flat3D& m_FlatShader;
		GL::Mesh m_Mesh;
		Object3D* m_Object;
		Matrix4 m_LocalPos;
	};

	class PhysicsRenderObject  {
	public:
		explicit PhysicsRenderObject(Shaders::Phong& shader, Shaders::Flat3D& fShader, SceneGraph::DrawableGroup3D& group, Scene3D& renderScene, const PhysicsPtr<IPhysicsObject>& physicsObject)
			: m_PhysicsObject(physicsObject)
		{
			m_Object = std::make_shared < Object3D>(&renderScene);
			std::vector<PhysicsPtr<IColliderGeometry>> geometries;
			std::vector<MathLib::HTransform3> transforms;
			bool isDynamic = physicsObject->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
			physicsObject->GetColliderGeometries(geometries,&transforms);
			for (size_t i = 0; i < geometries.size(); i++)
			{
				const auto& geometry = geometries[i];
				CollisionGeometryCreateOptions options;
				geometry->GetParams(options);

				Trade::MeshData meshData = Primitives::cubeSolid();
				auto* object = new Object3D{ &renderScene };
				Matrix4 matrix;
				if (!transforms.empty())
				{
					MathLib::HMatrix4 transposeMatrix = transforms[i].matrix().transpose();
					matrix = ToMagnum(transposeMatrix);
				}
				Matrix4 scalingMatrix= Matrix4::scaling(Vector3(1.f));
				switch (options.m_GeometryType)
				{
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
					meshData = Primitives::uvSphereSolid(6, 12); 
					scalingMatrix = Matrix4::scaling(ToMagnum(options.m_Scale) * options.m_SphereParams.m_Radius);
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
					meshData = Primitives::cubeSolid();
					scalingMatrix = Matrix4::scaling({ options.m_BoxParams.m_HalfExtents[0] * options.m_Scale[0],options.m_BoxParams.m_HalfExtents[1] * options.m_Scale[1],options.m_BoxParams.m_HalfExtents[2] * options.m_Scale[2] });
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
				{
					PhysicsMeshData newMeshData = GenerateCapsuleMeshData(options.m_CapsuleParams.m_Radius, options.m_CapsuleParams.m_HalfHeight, 8, 8);
					meshData = CreateMesh(newMeshData.m_Vertices, newMeshData.m_Indices);
					scalingMatrix = Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				}
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
					meshData = Primitives::planeSolid();
					object->rotateXLocal(90.0_degf);
					scalingMatrix = Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
					meshData = CreateMesh(options.m_TriangleMeshParams.m_Vertices, options.m_TriangleMeshParams.m_Indices);
					scalingMatrix = Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
					meshData = CreateMesh(options.m_ConvexMeshParams.m_Vertices, options.m_ConvexMeshParams.m_Indices);
					scalingMatrix = Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				default:
					continue;
				}
				matrix = matrix * scalingMatrix;
				object->setTransformation(matrix);
				object->setParent(m_Object.get());
				std::shared_ptr<RenderableObject> newObject = std::make_shared<RenderableObject>(object, shader,fShader, meshData, group , matrix);
				if(!isDynamic)
					newObject->SetAmbientColor(0x66000000_rgbaf);
				else
				{
					float randomFloat0 = float(rand()) / RAND_MAX;
					float randomFloat1 = float(rand()) / RAND_MAX;
					float randomFloat2 = float(rand()) / RAND_MAX;
					Color4 ambientColor = Color4(randomFloat0, randomFloat1, randomFloat2, 1.0f);
					newObject->SetAmbientColor(ambientColor);
				}
				newObject->SetDiffuseColor(0x55555555_rgbaf);
				m_RenderObjects.push_back(newObject);
			}

			{
				MathLib::HVector3 halfSize = physicsObject->GetLocalBoundingBox().sizes() / 2.f;
				Trade::MeshData meshData = Primitives::cubeWireframe();
				m_BoundingBoxObject = std::make_shared < Object3D>(&renderScene);
				m_BoundingBoxObject->scale(Vector3(halfSize[0], halfSize[1], halfSize[2]));
				m_ParentObject = m_BoundingBoxObject->parent();
				m_BoundingBoxObject->setParent(m_Object.get());
				m_BoundingBox = std::make_shared<FlatDrawable>(*m_BoundingBoxObject, fShader, meshData, group);				
				m_BoundingBox->SetColor(0x999999_rgbf);
			}
		}

		void UpdateTransform() {
			if ((m_PhysicsObject == nullptr && m_PhysicsObject->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC))
				return;
			const MathLib::HMatrix4& matrix = m_PhysicsObject->GetTransform().matrix();
			MathLib::HMatrix4 transposeMatrix = matrix.transpose();

			m_Object->setTransformation(ToMagnum(transposeMatrix));
			if(m_UseWorldBoundingBox)
			{
				MathLib::HVector3 halfSize = m_PhysicsObject->GetWorldBoundingBox().sizes() / 2.f;
				m_BoundingBoxObject->resetTransformation();
				m_BoundingBoxObject->scale(Vector3(halfSize[0], halfSize[1], halfSize[2]));
				auto meshData= Primitives::cubeWireframe();
				MathLib::HVector3 center = m_PhysicsObject->GetWorldBoundingBox().center();
				m_BoundingBox->SetMeshData(meshData);
				m_BoundingBoxObject->translate(ToMagnum(center));
			}
			IDynamicObject* dynamicObject = dynamic_cast<IDynamicObject*>(m_PhysicsObject.get());
			bool isSleeping = dynamicObject ? dynamicObject->IsSleeping() : false;
			for (auto& renderObject : m_RenderObjects)
			{
				renderObject -> SetIsSleeping(isSleeping);
			}
		}

		void ShowWireframe(bool show) {
			for (auto& renderObject : m_RenderObjects)
			{
				renderObject->ShowWireframe(show);
			}
		}

		void ShowBoundingBox(bool show) {
			if (m_BoundingBox == nullptr)
				return;
			m_bShowBoundingBox = show;
			m_BoundingBox->Show(show);
		}

		void Show(bool show)
		{
			for (auto& renderObject : m_RenderObjects)
				renderObject->Show(show);
			m_BoundingBox->Show(show&&m_bShowBoundingBox);
		}

		void UseWorldBoundingBox(bool useWorldBoundingBox)
		{
			if (m_UseWorldBoundingBox == useWorldBoundingBox)
				return;
			m_UseWorldBoundingBox = useWorldBoundingBox;
			if (m_UseWorldBoundingBox)
			{				
				m_BoundingBoxObject->setParent(m_ParentObject);
			}
			else
			{
				m_BoundingBoxObject->resetTransformation();
				MathLib::HVector3 halfSize = m_PhysicsObject->GetLocalBoundingBox().sizes() / 2.f;
				Trade::MeshData meshData = Primitives::cubeWireframe();
				m_BoundingBoxObject->scale(Vector3(halfSize[0], halfSize[1], halfSize[2]));
				m_BoundingBoxObject->setParent(m_Object.get());
			}
		}
	private:
		Object3D* m_ParentObject = nullptr;
		bool m_bShowBoundingBox = true;
		bool m_UseWorldBoundingBox = false;
		std::shared_ptr<FlatDrawable> m_BoundingBox;
		std::vector<std::shared_ptr<RenderableObject>> m_RenderObjects;
		PhysicsPtr<IPhysicsObject> m_PhysicsObject;
		std::shared_ptr < Object3D> m_BoundingBoxObject ;
		std::shared_ptr < Object3D> m_Object ;
	};
}