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
#include <Math/GraphicUtils/Camara.h>
#include "MagnumRender/MagnumConvertUtils.h"
#include "MeshDataLoader.h"
namespace MagnumRender
{
	static MathLib::HVector3 mLightPosition(7.0f, 5.0f, 2.5f);
	static Magnum::Color4 mLightColor(0.5f, 0.5f, 0.5f, 1.0f);
	using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;
	using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;
	using namespace Magnum::Math::Literals;

	class ShaderTable
	{
	public:
		ShaderTable()
		{
		}
		
		void Init()
		{
			if (m_bIsInitialized)
				return;
			m_FlatShader = Magnum::Shaders::Flat3D{};
			m_PhongShader = Magnum::Shaders::Phong{};
			m_bIsInitialized = true;
		}

		Magnum::Shaders::Flat3D& GetFlatShader()
		{
			Init();
			return m_FlatShader;
		}

		Magnum::Shaders::Phong& GetPhongShader()
		{
			Init();
			return m_PhongShader;
		}
	private:

			bool m_bIsInitialized = false;
			Magnum::Shaders::Flat3D m_FlatShader{ Magnum::NoCreate };
			Magnum::Shaders::Phong m_PhongShader{ Magnum::NoCreate };
	};

	static ShaderTable m_ShaderTable;
	class FlatDrawable {
	public:
		explicit FlatDrawable(const Magnum::Trade::MeshData& meshData) :
			m_Mesh(Magnum::MeshTools::compile(meshData)) {
		}

		void UpdateTransformation() {
			m_TransformationMatrix = m_Object.absoluteTransformationMatrix();
		}

		void ResetTransformation()
		{
			m_Object.resetTransformation();
			m_TransformationMatrix = m_Object.transformation();
		}

		void Render(MathLib::GraphicUtils::Camera& camera)
		{
			m_ShaderTable.GetFlatShader()
				.setColor(m_Color)
				.setTransformationProjectionMatrix(ToMagnum(camera.GetViewProjectMatrix()) *  m_TransformationMatrix)
				.draw(m_Mesh);
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

		void AddToScene(Object3D& scene)
		{
			m_Object.setParent(&scene);
			m_TransformationMatrix = m_Object.transformation();
		}

		void RemoveFromScene()
		{
			m_Object.setParent(nullptr);
		}

		Object3D& GetObject()
		{
			return m_Object;
		}

		void SetTransformation(const MathLib::HMatrix4& transform)
		{
			m_Object.setTransformation(ToMagnum(transform));
		}

	private:
		Object3D m_Object;
		bool m_bShow = true;
		Magnum::GL::Mesh m_Mesh;
		Magnum::Color4 m_Color = 0x747474_rgbf;
		Magnum::Matrix4 m_TransformationMatrix;
	};

	class RenderUnit {
	public:
		explicit RenderUnit(const Magnum::Trade::MeshData& meshData) :
			 m_Mesh(Magnum::MeshTools::compile(meshData))
		{
		}

		void UpdateTransformation()
		{
			m_TransformationMatrix = m_Object.absoluteTransformationMatrix();
		}

		void ResetTransformation()
		{
			m_Object.resetTransformation();
			m_TransformationMatrix = m_Object.transformation();
		}

		void Render(MathLib::GraphicUtils::Camera& camera) {
			if (!m_bShow)
				return;
			const Magnum::Matrix4& transformationMatrix = ToMagnum(camera.GetViewMatrix()) * m_TransformationMatrix;

			m_Mesh.setPrimitive(Magnum::MeshPrimitive::Triangles);
			m_ShaderTable.GetPhongShader().setLightPosition(ToMagnum(mLightPosition))
				.setAmbientColor(IsSleeping? m_AmbientColor*0.5:m_AmbientColor)
				.setDiffuseColor(m_DiffuseColor)
				.setLightColor(mLightColor)
				.setTransformationMatrix(transformationMatrix)
				.setNormalMatrix(transformationMatrix.normalMatrix())
				.setProjectionMatrix(ToMagnum(camera.GetProjectMatrix()));
			m_Mesh.draw(m_ShaderTable.GetPhongShader());
			if (m_bShowWireframe)
			{
				m_Mesh.setPrimitive(Magnum::MeshPrimitive::Lines);
				m_ShaderTable.GetFlatShader().setColor(0x000000_rgbf)
					.setTransformationProjectionMatrix(ToMagnum(camera.GetProjectMatrix()) * transformationMatrix)
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

		void AddToScene(Object3D& scene)
		{
			m_Object.setParent(&scene);	
			m_TransformationMatrix = m_Object.transformation();
		}

		void RemoveFromScene()
		{
			m_Object.setParent(nullptr);
		}

		void SetTransformation(const MathLib::HMatrix4& transform)
		{
			m_Object.resetTransformation();
			m_Object.setTransformation(ToMagnum(transform));
		}
	private:
		bool m_bShow = true;
		bool IsSleeping = false;
		bool m_bShowWireframe = true;
		Magnum::Color4 m_AmbientColor = { 1.f,1.f,1.f,1.f };
		Magnum::Color4 m_DiffuseColor = { 1.f,1.f,1.f,1.f };
		Magnum::GL::Mesh m_Mesh;
		Object3D m_Object;
		Magnum::Matrix4x4 m_TransformationMatrix;
	};

	class RenderObject  {
	public:
		explicit RenderObject(Magnum::SceneGraph::DrawableGroup3D& group, Scene3D& renderScene)
		{
			m_Object = std::make_shared < Object3D>(&renderScene);

			{
				MathLib::HVector3 halfSize(0, 0, 0);
				Magnum::Trade::MeshData meshData = Magnum::Primitives::cubeWireframe();
				m_BoundingBoxObject = std::make_shared < Object3D>(&renderScene);
				m_BoundingBoxObject->scale(Magnum::Vector3(halfSize[0], halfSize[1], halfSize[2]));
				m_Scene = &renderScene;
				m_BoundingBoxObject->setParent(m_Object.get());
				m_BoundingBox = std::make_shared<FlatDrawable>( meshData);				
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
				m_BoundingBoxObject->resetTransformation();
				m_BoundingBoxObject->setParent(m_Scene);
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
		Scene3D* m_Scene = nullptr;
		bool m_bShowBoundingBox = true;
		bool m_UseWorldBoundingBox = false;
		std::shared_ptr<FlatDrawable> m_BoundingBox;
		std::vector<std::shared_ptr<RenderUnit>> m_RenderObjects;
		std::shared_ptr < Object3D> m_BoundingBoxObject ;
		std::shared_ptr < Object3D> m_Object ;
		MathLib::HAABBox3D m_WorldBoundingBox;
		MathLib::HAABBox3D m_LocalBoundingBox;
	};
}