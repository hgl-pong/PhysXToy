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
namespace MagnumRender
{
	static MathLib::HVector3 mLightPosition(7.0f, 5.0f, 2.5f);
	static Magnum::Color4 mLightColor(0.5f, 0.5f, 0.5f, 1.0f);
	using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;
	using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;
	using namespace Magnum::Math::Literals;
	class FlatDrawable : public Magnum::SceneGraph::Drawable3D {
	public:
		explicit FlatDrawable(Object3D& object, Magnum::Shaders::Flat3D& shader, const Magnum::Trade::MeshData& meshData, Magnum::SceneGraph::DrawableGroup3D& drawables) :
			Magnum::SceneGraph::Drawable3D{ object, &drawables }, m_FlatShader(shader), m_Mesh(Magnum::MeshTools::compile(meshData)) {}

		void draw(const Magnum::Matrix4& transformation, Magnum::SceneGraph::Camera3D& camera) {
			if (m_bShow)
			{
				m_FlatShader
					.setColor(m_Color)
					.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
					.draw(m_Mesh);
			}
		}

		void Show(bool show) {
			m_bShow = show;
		}

		void SetColor(const Magnum::Color4& color) {
			m_Color = color;
		}

		void SetMeshData(Magnum::Trade::MeshData& meshdata)
		{
			m_Mesh = Magnum::MeshTools::compile(meshdata);
		}

	private:
		bool m_bShow = true;
		Magnum::Shaders::Flat3D& m_FlatShader;
		Magnum::GL::Mesh m_Mesh;
		Magnum::Color4 m_Color = 0x747474_rgbf;
	};

	class RenderableObject : public Magnum::SceneGraph::Drawable3D {
	public:
		explicit RenderableObject(Object3D* object, Magnum::Shaders::Phong& pshader, Magnum::Shaders::Flat3D&fShader, const Magnum::Trade::MeshData& meshData, Magnum::SceneGraph::DrawableGroup3D& group, const Magnum::Matrix4& matrix) :
			Magnum::SceneGraph::Drawable3D{ *object, &group }, m_PhongShader(pshader),m_FlatShader(fShader),m_Mesh(Magnum::MeshTools::compile(meshData)), m_Object(object ) , m_LocalPos(matrix){}

		void draw(const Magnum::Matrix4& transformationMatrix, Magnum::SceneGraph::Camera3D& camera) override {
			if (!m_bShow)
				return;
			m_Mesh.setPrimitive(Magnum::MeshPrimitive::Triangles);
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
				m_Mesh.setPrimitive(Magnum::MeshPrimitive::Lines);
				m_FlatShader.setColor(0x000000_rgbf)
					.setTransformationProjectionMatrix(camera.projectionMatrix() * transformationMatrix)
					.draw(m_Mesh);
			}
		}

		void SetAmbientColor(const Magnum::Color4& color) {
			m_AmbientColor = color;
		}
		void SetDiffuseColor(const Magnum::Color4& color) {
			m_DiffuseColor = color;
		}

		void SetIsSleeping(bool isSleeping)
		{
			IsSleeping = isSleeping;
		}

		void ShowWireframe(bool show) {
			m_bShowWireframe = show;
		}

		void SetMeshData(Magnum::Trade::MeshData& meshdata)
		{
			m_Mesh = Magnum::MeshTools::compile(meshdata);
		}

		void Show(bool show)
		{
			m_bShow = show;
		}
	private:
		bool m_bShow = true;
		bool IsSleeping = false;
		bool m_bShowWireframe = true;
		Magnum::Color4 m_AmbientColor = { 1.f,1.f,1.f,1.f };
		Magnum::Color4 m_DiffuseColor = { 1.f,1.f,1.f,1.f };

		Magnum::Shaders::Phong& m_PhongShader;
		Magnum::Shaders::Flat3D& m_FlatShader;
		Magnum::GL::Mesh m_Mesh;
		Object3D* m_Object;
		Magnum::Matrix4 m_LocalPos;
	};

	class RenderObject  {
	public:
		explicit RenderObject(Magnum::Shaders::Phong& shader, Magnum::Shaders::Flat3D& fShader, Magnum::SceneGraph::DrawableGroup3D& group, Scene3D& renderScene)
		{
			m_Object = std::make_shared < Object3D>(&renderScene);

			{
				MathLib::HVector3 halfSize(0, 0, 0);
				Magnum::Trade::MeshData meshData = Magnum::Primitives::cubeWireframe();
				m_BoundingBoxObject = std::make_shared < Object3D>(&renderScene);
				m_BoundingBoxObject->scale(Magnum::Vector3(halfSize[0], halfSize[1], halfSize[2]));
				m_ParentObject = m_BoundingBoxObject->parent();
				m_BoundingBoxObject->setParent(m_Object.get());
				m_BoundingBox = std::make_shared<FlatDrawable>(*m_BoundingBoxObject, fShader, meshData, group);				
				m_BoundingBox->SetColor(0x999999_rgbf);
			}
		}

		virtual void UpdateTransform() {
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
				m_BoundingBoxObject->setParent(m_Object.get());
			}
		}

		MathLib::HAABBox3D GetWorldBoundingBox() const
		{
			return m_WorldBoundingBox;
		}
	private:
		Object3D* m_ParentObject = nullptr;
		bool m_bShowBoundingBox = true;
		bool m_UseWorldBoundingBox = false;
		std::shared_ptr<FlatDrawable> m_BoundingBox;
		std::vector<std::shared_ptr<RenderableObject>> m_RenderObjects;
		std::shared_ptr < Object3D> m_BoundingBoxObject ;
		std::shared_ptr < Object3D> m_Object ;
		MathLib::HAABBox3D m_WorldBoundingBox;
		MathLib::HAABBox3D m_LocalBoundingBox;
	};
}