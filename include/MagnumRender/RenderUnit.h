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
			m_VertexColorShader = Magnum::Shaders::VertexColor3D{};
			m_bIsInitialized = true;
		}

		Magnum::Shaders::Flat3D &GetFlatShader()
		{
			Init();
			return m_FlatShader;
		}

		Magnum::Shaders::Phong &GetPhongShader()
		{
			Init();
			return m_PhongShader;
		}

		Magnum::Shaders::VertexColor3D &GetVertexColorShader()
		{
			Init();
			return m_VertexColorShader;
		}

	private:
		bool m_bIsInitialized = false;
		Magnum::Shaders::Flat3D m_FlatShader{Magnum::NoCreate};
		Magnum::Shaders::Phong m_PhongShader{Magnum::NoCreate};
		Magnum::Shaders::VertexColor3D m_VertexColorShader{Magnum::NoCreate};
	};

	static ShaderTable m_ShaderTable;

	class RenderUnitBase
	{
	public:
		explicit RenderUnitBase(const Magnum::Trade::MeshData &meshData)
			: m_Mesh(Magnum::MeshTools::compile(meshData))
		{
		}

		virtual void UpdateTransformation()
		{
			Object3D *parent = m_Object.parent();
			if (parent == nullptr)
				return;
			auto t = parent->absoluteTransformationMatrix();
			m_TransformationMatrix = m_Object.absoluteTransformationMatrix();
		};
		virtual void ResetTransformation()
		{
			m_Object.resetTransformation();
			UpdateTransformation();
		};
		virtual void Render(MathLib::GraphicUtils::Camera &camera) {}
		virtual void Show(bool show)
		{
			m_bShow = show;
		}
		virtual void SetMeshData(const Magnum::Trade::MeshData &meshdata) { m_Mesh = Magnum::MeshTools::compile(meshdata); }
		virtual void AddToScene(Object3D &scene)
		{
			m_Object.setParent(&scene);
			UpdateTransformation();
		}
		virtual void RemoveFromScene()
		{
			m_Object.setParent(nullptr);
		}
		virtual void SetTransformation(const MathLib::HMatrix4 &transform)
		{
			m_Object.setTransformation(ToMagnum(transform));
			UpdateTransformation();
		}

		Object3D &GetObject()
		{
			return m_Object;
		}

		void SetTransformation(const MathLib::HVector3 *scaling = nullptr, const MathLib::HVector3 *translation = nullptr, const MathLib::HQuaternion *rotation = nullptr)
		{
			Magnum::Matrix4 currentTransform = m_Object.transformationMatrix();
			Magnum::Matrix4 scalingMatrix = scaling == nullptr ? Magnum::Matrix4::scaling(currentTransform.scaling()) : Magnum::Matrix4::scaling(ToMagnum(*scaling));
			Magnum::Matrix4 rotationMatrix = rotation == nullptr ? Magnum::Matrix4::from(currentTransform.rotation(), {}) : Magnum::Matrix4::from(ToMagnum(*rotation).toMatrix(), {});
			Magnum::Vector3 translationVector = translation == nullptr ? currentTransform.translation() : ToMagnum(*translation);

			m_Object.resetTransformation();
			m_Object.setTransformation(scalingMatrix * rotationMatrix);
			m_Object.translate(translationVector);

			UpdateTransformation();
		}

	protected:
		bool m_bShow = true;
		Magnum::GL::Mesh m_Mesh;
		Object3D m_Object;
		Magnum::Matrix4 m_TransformationMatrix;
	};

	class GizmoRenderUnit : public RenderUnitBase
	{
	public:
		explicit GizmoRenderUnit(const Magnum::Trade::MeshData &meshData)
			: RenderUnitBase(meshData)
		{
		}

		void Render(MathLib::GraphicUtils::Camera &camera) override
		{
			if (m_bShow)
			{
				m_ShaderTable.GetFlatShader()
					.setColor(m_Color)
					.setTransformationProjectionMatrix(ToMagnum(camera.GetViewProjectMatrix()) * m_TransformationMatrix)
					.draw(m_Mesh);
			}
		}

		void SetColor(const Magnum::Color4 &color)
		{
			m_Color = color;
		}

	private:
		Magnum::Color4 m_Color = 0x747474_rgbf;
	};

	class SimpleRenderUnit : public RenderUnitBase
	{
	public:
		explicit SimpleRenderUnit(const Magnum::Trade::MeshData &meshData)
			: RenderUnitBase(meshData)
		{
		}

		void Render(MathLib::GraphicUtils::Camera &camera) override
		{
			if (!m_bShow)
				return;
			const Magnum::Matrix4 &transformationMatrix = ToMagnum(camera.GetViewMatrix()) * m_TransformationMatrix;

			m_Mesh.setPrimitive(Magnum::MeshPrimitive::Triangles);
			m_ShaderTable.GetPhongShader().setLightPosition(ToMagnum(mLightPosition)).setAmbientColor(m_AmbientColor).setDiffuseColor(m_DiffuseColor).setLightColor(mLightColor).setTransformationMatrix(transformationMatrix).setNormalMatrix(transformationMatrix.normalMatrix()).setProjectionMatrix(ToMagnum(camera.GetProjectMatrix()));
			m_Mesh.draw(m_ShaderTable.GetPhongShader());
			if (m_bShowWireframe)
			{
				m_Mesh.setPrimitive(Magnum::MeshPrimitive::LineStrip);
				m_ShaderTable.GetFlatShader().setColor(0x000000_rgbf).setTransformationProjectionMatrix(ToMagnum(camera.GetProjectMatrix()) * transformationMatrix).draw(m_Mesh);
			}
		}

		void SetAmbientColor(const Magnum::Color4 &color)
		{
			m_AmbientColor = color;
		}

		Magnum::Color4 GetAmbientColor() const
		{
			return m_AmbientColor;
		}

		void SetDiffuseColor(const Magnum::Color4 &color)
		{
			m_DiffuseColor = color;
		}

		Magnum::Color4 GetDiffuseColor() const
		{
			return m_DiffuseColor;
		}

		void ShowWireframe(bool show)
		{
			m_bShowWireframe = show;
		}

	private:
		bool m_bShowWireframe = true;
		Magnum::Color4 m_AmbientColor = {1.f, 1.f, 1.f, 1.f};
		Magnum::Color4 m_DiffuseColor = {1.f, 1.f, 1.f, 1.f};
	};

	class CoordinateAxis : public RenderUnitBase
	{
	public:
		explicit CoordinateAxis()
			: RenderUnitBase(CreateMesh(MathLib::GraphicUtils::GenerateBoxMeshData(MathLib::HVector3(1, 1, 1))))
		{
			const Vertex data[] = {
				// X axis (red)
				{{0.0f, 0.0f, 0.0f}, 0xff0000_rgbf},
				{{100.0f, 0.0f, 0.0f}, 0xff0000_rgbf},
				// Y axis (green)
				{{0.0f, 0.0f, 0.0f}, 0x00ff00_rgbf},
				{{0.0f, 100.0f, 0.0f}, 0x00ff00_rgbf},
				// Z axis (blue)
				{{0.0f, 0.0f, 0.0f}, 0x0000ff_rgbf},
				{{0.0f, 0.0f, 100.0f}, 0x0000ff_rgbf}};

			m_VertexBuffer.setData(data);
			m_Mesh = Magnum::GL::Mesh{};
			m_Mesh.setPrimitive(Magnum::GL::MeshPrimitive::Lines)
				.setCount(6)
				.addVertexBuffer(std::move(m_VertexBuffer), 0, Magnum::Shaders::VertexColor3D::Position{}, Magnum::Shaders::VertexColor3D::Color3{});
		}

		void Render(MathLib::GraphicUtils::Camera &camera) override
		{

			if (m_bShow)
			{
				Magnum::Matrix4 viewProjectionMatrix = ToMagnum(camera.GetViewProjectMatrix());

				Magnum::Matrix4 screenTransform = Magnum::Matrix4::translation({-0.9f, -0.9f, 0.0f}) *
												  Magnum::Matrix4::scaling({0.1f, 0.1f, 0.1f});

				m_ShaderTable.GetVertexColorShader().setTransformationProjectionMatrix(viewProjectionMatrix * m_TransformationMatrix);
				m_Mesh.draw(m_ShaderTable.GetVertexColorShader());
			}
		}

	private:
		struct Vertex
		{
			Magnum::Vector3 position;
			Magnum::Color3 color;
		};
		Magnum::GL::Buffer m_VertexBuffer;
	};
}