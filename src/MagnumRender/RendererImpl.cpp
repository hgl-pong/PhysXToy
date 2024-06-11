#include "RendererImpl.h"
#include "MagnumRender/MagnumRenderCommon.h"
#include "MagnumRender/MagnumConvertUtils.h"
#include "MagnumRender/RenderUnit.h"
using namespace Magnum;
using namespace Math::Literals;
namespace MagnumRender
{
	RendererImpl::RendererImpl(const Arguments& arguments) : Platform::Application{ arguments, NoCreate } {
		/* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
		   MSAA if we have enough DPI. */
		{
			const Vector2 dpiScaling = this->dpiScaling({});
			Configuration conf;
			conf.setTitle(m_ApplicationName)
				.setSize(conf.size(), dpiScaling);
			GLConfiguration glConf;
			glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
			if (!tryCreate(conf, glConf))
				create(conf, glConf.setSampleCount(0));

		}
		m_CameraManager.CreateCamrera("Camera0", MathLib::HVector3(80.0f, 80.0f, 80.0f), MathLib::HVector3(-0.6f, -0.6f, -0.6f), MathLib::HReal(Vector2{ framebufferSize() }.aspectRatio()));
		m_CameraManager.CreateCamrera("Camera1", MathLib::HVector3(80.0f, 80.0f, 80.0f), MathLib::HVector3(-0.6f, -0.6f, -0.6f), MathLib::HReal(Vector2{ framebufferSize() }.aspectRatio()));

		m_FrustumCullingManager = std::make_unique<MathLib::GraphicUtils::CullingManager>(*m_CameraManager.GetActiveCamera());
		m_FrustumCullingManager->SetCullingDistance(200);

		GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
		GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);

		/* Grid */
		m_GridMesh = std::make_unique<MagnumRender::GizmoRenderUnit>(Primitives::grid3DWireframe({ 1500, 1500 }));
		m_GridMesh->GetObject().rotateX(90.0_degf)
			.scale(Vector3{ 800.0f });
		m_GridMesh->AddToScene(m_RenderScene);

		m_CoordinateAxis = std::make_unique<MagnumRender::CoordinateAxis>();
		m_CoordinateAxis->AddToScene(m_RenderScene);

		m_FrustumCullingManager->UpdateFrustum();
	}

	void RendererImpl::SetUp(const MousePresseCallback& mousePressCallback,
		const MouseReleaseCallback& mouseReleaseCallback,
		const MouseMotionCallback& mouseMotioCallback,
		const MouseScrollCallback& mouseScrollCallback,
		const KeyBoardPressCallback& keyboardPressCallback,
		const KeyBoardReleaseCallback& keyBoardCallback)
	{
		m_MousePressCallback = mousePressCallback;
		m_MouseReleaseCallback = mouseReleaseCallback;
		m_MouseMotionCallback = mouseMotioCallback;
		m_MouseScrollCallback = mouseScrollCallback;
		m_KeyBoardPressCallback = keyboardPressCallback;
		m_KeyBoardReleaseCallback = keyBoardCallback;
	}

	void RendererImpl::SetApplicationName(const char* name)
	{
		setWindowTitle(name);
		m_ApplicationName = name;
	}

	void RendererImpl::keyPressEvent(KeyEvent& event) {
		/* Reset the transformation to the original view */
		char cameraKey = '\0';
		switch (event.key())
		{
		case KeyEvent::Key::W:
			cameraKey = 'W';
			break;
		case KeyEvent::Key::S:
			cameraKey = 'S';
			break;
		case KeyEvent::Key::A:
			cameraKey = 'A';
			break;
		case KeyEvent::Key::D:
			cameraKey = 'D';
			break;
		case KeyEvent::Key::Q:
			cameraKey = 'Q';
			break;
		case KeyEvent::Key::E:
			cameraKey = 'E';
			break;
		}
		if (m_CameraManager.GetActiveCamera()->HandleKey(cameraKey, 0, 0))
		{
			m_FrustumCullingManager->UpdateFrustum();
		}
		m_KeyBoardPressCallback(event);
	}

	void RendererImpl::keyReleaseEvent(KeyEvent& event)
	{
		char cameraKey = '\0';
		switch (event.key())
		{
		case KeyEvent::Key::W:
			cameraKey = 'W';
			break;
		case KeyEvent::Key::S:
			cameraKey = 'S';
			break;
		case KeyEvent::Key::A:
			cameraKey = 'A';
			break;
		case KeyEvent::Key::D:
			cameraKey = 'D';
			break;
		case KeyEvent::Key::P:
		{
			m_CameraManager.SwitchToNextCamera();
			break;
		}
		default:
			break;
		}
		m_CameraManager.GetActiveCamera()->HandleAnalogMove(0, 0);
		//m_RenderCameraObject->setTransformation(ToMagnumMatrix4(m_MainCamera->getTransform().matrix()));
		m_FrustumCullingManager->UpdateFrustum();
		m_KeyBoardReleaseCallback(event);
	}

	void RendererImpl::mousePressEvent(MouseEvent& event) {
		if (event.button() != MouseEvent::Button::Left &&
			event.button() != MouseEvent::Button::Middle)
			return;
		if (m_CameraManager.GetActiveCamera())
			m_CameraManager.GetActiveCamera()->HandleMouse(0, 0, event.position().x(), event.position().y());
		m_MousePressCallback(event);
	}

	void RendererImpl::mouseMoveEvent(MouseMoveEvent& event) {

		if (!event.buttons()) return;

		if (m_CameraManager.GetActiveCamera())
			m_CameraManager.GetActiveCamera()->HandleMotion(event.position().x(), event.position().y());

		//m_RenderCameraObject->setTransformation(ToMagnumMatrix4(m_MainCamera->getTransform().matrix()));
		m_FrustumCullingManager->UpdateFrustum();

		//printf("Dir:%f,%f,%f\n", m_MainCamera->getDir().x(), m_MainCamera->getDir().y(), m_MainCamera->getDir().z());
		m_MouseMotionCallback(event);
	}

	void RendererImpl::mouseScrollEvent(MouseScrollEvent& event) {
		event.setAccepted();
	}

	void RendererImpl::drawEvent() {
		MathLib::GraphicUtils::Camera* camera = m_CameraManager.GetActiveCamera();
		if (camera == nullptr)
			return;
		m_FrameProfiler.Start();
		GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
		for (auto& renderable : m_RenderObjects)
		{
			renderable->UpdateTransform();
			bool show = !m_FrustumCullingManager->CullingObject(renderable->GetWorldBoundingBox());
			renderable->Show(show);
			renderable->Render(*camera);
		}
		m_GridMesh->Render(*camera);
		m_CoordinateAxis->Render(*camera);

		m_FrameProfiler.End();
		char title[256];
		sprintf(title, "%s %.1f FPS %.1f ms", m_ApplicationName.c_str(), m_FrameProfiler.GetFrameRate(), m_FrameProfiler.GetFrameTime());
		setWindowTitle(title);
		swapBuffers();
		redraw();
	}

	void RendererImpl::AddRenderObject(std::shared_ptr<RenderObject>& renderObject)
	{
		m_RenderObjects.emplace(renderObject);
		renderObject->AddToScene(m_RenderScene);
	}

	void RendererImpl::RemoveRenderObject(std::shared_ptr<RenderObject>& renderObject)
	{
		m_RenderObjects.erase(renderObject);
		renderObject->RemoveFromScene();
	}

	MathLib::GraphicUtils::Camera* RendererImpl::GetActiveCamera()
	{
		return m_CameraManager.GetActiveCamera();
	}

	Renderer* CreateRenderer(int argc, char** argv)
	{
		return new RendererImpl({ argc, argv });
	}
}