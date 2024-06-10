#pragma once
#include <MagnumRender/MagnumRenderCommon.h>
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

#include <Math/GraphicUtils/Frustum.h>
#include <Math/GraphicUtils/CameraManager.h>
#include <Math/GraphicUtils/FrameProfiler.h>
#include <unordered_set>
namespace MagnumRender
{
	using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;
	using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;
	class RenderObject;
	class RendererImpl : public Magnum::Platform::Application, virtual public Renderer {
	public:
		explicit RendererImpl(const Arguments& arguments);
	public:
		void SetUp(MousePresseCallback& mousePressCallback,
			MouseReleaseCallback& mouseReleaseCallback,
			MouseMotionCallback& mouseMotioCallback,
			MouseScrollCallback& mouseScrollCallback,
			KeyBoardPressCallback& keyboardPressCallback,
			KeyBoardReleaseCallback& keyBoardCallback)override;
		void Release() override {
			delete this;
		}
		int Run() override {
			return exec();
		}
		void  SetApplicationName(const char* name)override;
	private:
		void keyPressEvent(KeyEvent& event) override;
		void keyReleaseEvent(KeyEvent& event)override;
		void mousePressEvent(MouseEvent& event) override;
		void mouseMoveEvent(MouseMoveEvent& event) override;
		void mouseScrollEvent(MouseScrollEvent& event) override;
		void drawEvent() override;

		void AddRenderObjecct(std::shared_ptr<RenderObject>& renderObject);
		void RemoveRenderObject(std::shared_ptr<RenderObject>& renderobject);

	private:
		Magnum::Shaders::Flat3D m_FlatShader{ Magnum::NoCreate };
		Magnum::Shaders::Phong m_PhongShader{ Magnum::NoCreate };

		std::string m_ApplicationName = "Magnum Renderer";

		Scene3D m_RenderScene;
		Magnum::SceneGraph::DrawableGroup3D m_RenderDrawable;
		Object3D* m_RenderCameraObject;
		Magnum::SceneGraph::Camera3D* m_RenderCamera;
		Object3D* m_GridObject = nullptr;

		std::unordered_set<std::shared_ptr<RenderObject>> m_RenderObjects;

		MathLib::GraphicUtils::FrameProfiler m_FrameProfiler;
		MathLib::GraphicUtils::CameraManager m_CameraManager;
		std::unique_ptr<MathLib::GraphicUtils::CullingManager> m_FrustumCullingManager;

		std::function<void(MouseEvent&)> m_MousePressCallback;
		std::function<void(MouseEvent&)> m_MouseReleaseCallback;
		std::function<void(MouseMoveEvent&)> m_MouseMotionCallback;
		std::function<void(MouseScrollEvent&)> m_MouseScrollCallback;
		std::function<void(KeyEvent&)> m_KeyBoardPressCallback;
		std::function<void(KeyEvent&)> m_KeyBoardReleaseCallback;
	};
}