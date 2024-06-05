#include "Application.h"
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

#include "Physics/PhysicsCommon.h"
#include "PxPhysicsAPI.h"

#include "TestRigidBodyCreate.h"
#include "MagnumConvertUtils.h"
#include "PhysicsRenderObject.h"
#include "FrameProfiler.h"
#include "Camera.h"
using namespace physx;
namespace Magnum {

		using namespace Math::Literals;

		class TestingApplication : public Platform::Application ,virtual public PhysicsEngineTestingApplication  {
		public:
			explicit TestingApplication(const Arguments& arguments);
		public:
			void Release() override {
				m_Scene.reset();
				m_Material.reset();
				PhysicsEngineUtils::DestroyPhysicsEngine();
				delete this;
			}
			int Run() override {
				return exec();
			}
		private:
			void keyPressEvent(KeyEvent& event) override;
			void keyReleaseEvent(KeyEvent& event)override;
			void mousePressEvent(MouseEvent& event) override;
			void mouseMoveEvent(MouseMoveEvent& event) override;
			void mouseScrollEvent(MouseScrollEvent& event) override;
			void drawEvent() override;

		private:
			void _InitPhysics(bool interactive);
			void _AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject>& object);
			PhysicsPtr<IPhysicsObject> _CreateDynamic(const MathLib::HTransform3& t, PhysicsPtr < IColliderGeometry>& geometry, const MathLib::HVector3& velocity = MathLib::HVector3(0, 0, 0));
			private:
			Shaders::Flat3D m_FlatShader{ NoCreate };
			Shaders::Phong m_PhongShader{ NoCreate };

			Scene3D m_RenderScene;
			SceneGraph::DrawableGroup3D m_RenderDrawable;
			Object3D* m_RenderCameraObject;
			SceneGraph::Camera3D* m_RenderCamera;
			Object3D* m_GridObject = nullptr;

			std::vector<std::shared_ptr<PhysicsRenderObject>> m_DynamicRenderableObjects;

			PhysicsPtr < IPhysicsMaterial>m_Material;
			PhysicsPtr < IPhysicsScene>m_Scene;
			std::unique_ptr<MathLib::Camera> m_Camera;
			FrameProfiler m_FrameProfiler;
		};

		TestingApplication::TestingApplication(const Arguments& arguments) : Platform::Application{ arguments, NoCreate } {
			/* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
			   MSAA if we have enough DPI. */
			{
				const Vector2 dpiScaling = this->dpiScaling({});
				Configuration conf;
				conf.setTitle("Testing PhysicsEngine")
					.setSize(conf.size(), dpiScaling);
				GLConfiguration glConf;
				glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
				if (!tryCreate(conf, glConf))
					create(conf, glConf.setSampleCount(0));			

			}
			m_Camera = std::make_unique<MathLib::Camera>(MathLib::HVector3(80.0f, 80.0f, 80.0f), MathLib::HVector3(-0.6f, -0.6f, -0.6f), MathLib::HReal(Vector2{ framebufferSize()}.aspectRatio()));
			/* Shaders, renderer setup */
			m_FlatShader = Shaders::Flat3D{};
			m_PhongShader = Shaders::Phong{};
			GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

			/* Grid */
			m_GridObject = new Object3D{ &m_RenderScene };
			(*m_GridObject)
				.rotateX(90.0_degf)
				.scale(Vector3{ 800.0f });
			new FlatDrawable{ *m_GridObject, m_FlatShader, Primitives::grid3DWireframe({ 1500, 1500 }), m_RenderDrawable };

			/* Set up the camera */
			m_RenderCameraObject = new Object3D{ &m_RenderScene };
			m_RenderCameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));

			m_RenderCamera = new SceneGraph::Camera3D{ *m_RenderCameraObject };

			m_RenderCamera->setProjectionMatrix(ToMagnum(m_Camera->getProjectMatrix()));

			_InitPhysics(false);
		}

		void TestingApplication::keyPressEvent(KeyEvent& event) {
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
			if(m_Camera->handleKey(cameraKey,0,0))
				m_RenderCameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));
		}

		void TestingApplication::keyReleaseEvent(KeyEvent& event)
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
			case KeyEvent::Key::Space:
			{
				CollisionGeometryCreateOptions options;
				options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
				options.m_SphereParams.m_Radius = 2.0f;

				PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

				_CreateDynamic(m_Camera->getTransform(), geometry, m_Camera->getDir() * 75);
				break;
			}
			case KeyEvent::Key::B:
			{
				auto physicsObject = TestRigidBody::TestRigidBodyCreate();
				if (m_Scene)
				{
					for (auto& physicsObject : physicsObject)
					{
						m_Scene->AddPhysicsObject(physicsObject);
						_AddPhysicsDebugRenderableObject(physicsObject);
					}
				}
				break;
			}
			default:
				break;
			}
			m_Camera->handleAnalogMove(0, 0);
			//m_RenderCameraObject->setTransformation(ToMagnumMatrix4(m_Camera->getTransform().matrix()));
			m_RenderCameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));

		}

		void TestingApplication::mousePressEvent(MouseEvent& event) {
			if (event.button() != MouseEvent::Button::Left &&
				event.button() != MouseEvent::Button::Middle)
				return;
			if (m_Camera.get())
				m_Camera->handleMouse(0,0,event.position().x(),event.position().y());
		}

		void TestingApplication::mouseMoveEvent(MouseMoveEvent& event) {

			if (!event.buttons()) return;

			if (m_Camera.get())
				m_Camera->handleMotion(event.position().x(), event.position().y());

			//m_RenderCameraObject->setTransformation(ToMagnumMatrix4(m_Camera->getTransform().matrix()));
			m_RenderCameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));
			//printf("Dir:%f,%f,%f\n", m_Camera->getDir().x(), m_Camera->getDir().y(), m_Camera->getDir().z());

		}

		void TestingApplication::mouseScrollEvent(MouseScrollEvent& event) {
			event.setAccepted();
		}

		void TestingApplication::drawEvent() {
			m_FrameProfiler.Start();

			GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
			m_Scene->Tick(1.f/10.f);
			//{
			//	const Matrix4 transformation = m_RenderCameraObject->transformationMatrix();
			//	const Vector3 translation = transformation.translation();
			//	Matrix4 gridMatrix = m_GridObject->transformation();
			//	auto rotate =gridMatrix.rotation();
			//	auto scaling = gridMatrix.scaling();
			//	gridMatrix =Matrix4::scaling(scaling)*Matrix4::from(rotate, { translation.x(), gridMatrix.translation().y(), translation.z() }) ;
			//	m_GridObject->setTransformation(gridMatrix);	
			//}
			for (auto& renderable : m_DynamicRenderableObjects)
				renderable->UpdateTransform();
			m_RenderCamera->draw(m_RenderDrawable);	
			m_FrameProfiler.End();
			char title[256];
			sprintf(title, "Testing PhysicsEngine %.1f FPS %.1f ms", m_FrameProfiler.GetFrameRate(), m_FrameProfiler.GetFrameTime());
			setWindowTitle(title);
			swapBuffers();
			redraw();
		}

		void TestingApplication::_InitPhysics(bool interactive)
		{
			PhysicsEngineOptions options;
			options.m_iNumThreads = 10;
			IPhysicsEngine* engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

			PhysicsSceneCreateOptions sceneOptions;
			sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
			sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

			m_Scene = PhysicsEngineUtils::CreateScene(sceneOptions);

			PhysicsMaterialCreateOptions materialOptions;
			materialOptions.m_StaticFriction = 0.5f;
			materialOptions.m_DynamicFriction = 0.5f;
			materialOptions.m_Restitution = 0.6f;
			materialOptions.m_Density = 10.0f;
			m_Material = PhysicsEngineUtils::CreateMaterial(materialOptions);

			CollisionGeometryCreateOptions groundPlaneOptions;
			groundPlaneOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE;
			groundPlaneOptions.m_PlaneParams.m_Normal = MathLib::HVector3(0, 1, 0);
			groundPlaneOptions.m_PlaneParams.m_Distance = 0.0f;
			PhysicsPtr<IColliderGeometry> groundPlane = PhysicsEngineUtils::CreateColliderGeometry(groundPlaneOptions);

			PhysicsObjectCreateOptions groundPlaneObjectOptions;
			groundPlaneObjectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
			groundPlaneObjectOptions.m_Transform = MathLib::HTransform3::Identity();
			PhysicsPtr < IPhysicsObject> groundPlaneObject = PhysicsEngineUtils::CreateObject(groundPlaneObjectOptions);
			groundPlaneObject->AddColliderGeometry(groundPlane, MathLib::HTransform3::Identity());
			if (m_Scene)
				m_Scene->AddPhysicsObject(groundPlaneObject);
			//_AddPhysicsDebugRenderableObject(groundPlaneObject);

			TestRigidBody::CreateTestingMeshData();//Bunny
			//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\teapot.obj", 0.2);
			//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\banana.obj", 1);
			//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\armadillo.obj",0.4);
			auto physicsObject= TestRigidBody::TestRigidBodyCreate();
			if (m_Scene)
			{
				for (auto& physicsObject : physicsObject)
				{
					m_Scene->AddPhysicsObject(physicsObject);
					_AddPhysicsDebugRenderableObject(physicsObject);
				}
			}

			if (!interactive)
			{
				CollisionGeometryCreateOptions options;
				options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
				options.m_SphereParams.m_Radius = 10.0f;
				options.m_Scale = MathLib::HVector3(1.0f, 1.0f, 1.0f);

				PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

				MathLib::HVector3 translation(0, 40, 100);
				MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
				transform.translate(translation);
				_CreateDynamic(transform, geometry, MathLib::HVector3(0, -50, -100));
			}
		}

		void TestingApplication::_AddPhysicsDebugRenderableObject(const PhysicsPtr<IPhysicsObject>& physicsObject)
		{
			std::shared_ptr<PhysicsRenderObject> renderable = std::make_shared<PhysicsRenderObject>(m_PhongShader,m_FlatShader, m_RenderDrawable, m_RenderScene, physicsObject);
			m_DynamicRenderableObjects.push_back(renderable);
		}
		PhysicsPtr<IPhysicsObject> TestingApplication::_CreateDynamic(const MathLib::HTransform3& t, PhysicsPtr < IColliderGeometry>& geometry, const MathLib::HVector3& velocity )
		{
			auto dynamic = TestRigidBody::CreateDynamic(t, geometry, velocity);
			if (m_Scene)
				m_Scene->AddPhysicsObject(dynamic);
			_AddPhysicsDebugRenderableObject(dynamic);
			return dynamic;
		}
}

PhysicsEngineTestingApplication* CreatePhysicsEngineTestingApplication(int argc, char** argv)
{
	return new Magnum::TestingApplication({ argc, argv });
}
