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
using namespace physx;

namespace Magnum {
		using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
		using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;

		using namespace Math::Literals;

		class TestingApplication : public Platform::Application ,virtual public PhysicsEngineTestingApplication  {
		public:
			explicit TestingApplication(const Arguments& arguments);
		public:
			void Release() override {
				delete this;
			}
			void Run() override {
				exec();
			}
		private:
			Float depthAt(const Vector2i& windowPosition);
			Vector3 unproject(const Vector2i& windowPosition, Float depth) const;

			void keyPressEvent(KeyEvent& event) override;
			void mousePressEvent(MouseEvent& event) override;
			void mouseMoveEvent(MouseMoveEvent& event) override;
			void mouseScrollEvent(MouseScrollEvent& event) override;
			void drawEvent() override;

		private:
			void renderActors(PxRigidActor** actors, const PxU32 numActors, bool shadows, const PxVec3& color, bool changeColorForSleepingActors, bool wireframePass);
			private:
			Shaders::VertexColor3D _vertexColorShader{ NoCreate };
			Shaders::Flat3D _flatShader{ NoCreate };
			Shaders::Phong _phongShader{ NoCreate };
			GL::Mesh _mesh{ NoCreate }, _grid{ NoCreate };

			Scene3D _scene;
			SceneGraph::DrawableGroup3D _drawables;
			Object3D* _cameraObject;
			SceneGraph::Camera3D* _camera;

			Float _lastDepth;
			Vector2i _lastPosition{ -1 };
			Vector3 _rotationPoint, _translationPoint;
		};

		class VertexColorDrawable : public SceneGraph::Drawable3D {
		public:
			explicit VertexColorDrawable(Object3D& object, Shaders::VertexColor3D& shader, GL::Mesh& mesh, SceneGraph::DrawableGroup3D& drawables) : SceneGraph::Drawable3D{ object, &drawables }, _shader(shader), _mesh(mesh) {}

			void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
				_shader
					.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
					.draw(_mesh);
			}

		private:
			Shaders::VertexColor3D& _shader;
			GL::Mesh& _mesh;
		};

		class FlatDrawable : public SceneGraph::Drawable3D {
		public:
			explicit FlatDrawable(Object3D& object, Shaders::Flat3D& shader, GL::Mesh& mesh, SceneGraph::DrawableGroup3D& drawables) : SceneGraph::Drawable3D{ object, &drawables }, _shader(shader), _mesh(mesh) {}

			void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
				_shader
					.setColor(0x747474_rgbf)
					.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
					.draw(_mesh);
			}

		private:
			Shaders::Flat3D& _shader;
			GL::Mesh& _mesh;
		};

		class RenderableObject : public SceneGraph::Drawable3D {
		public:
			explicit RenderableObject(Object3D& object, Shaders::Phong& shader, const Trade::MeshData& meshData, SceneGraph::DrawableGroup3D& group) :
				SceneGraph::Drawable3D{ object, &group }, _shader(shader), _mesh(MeshTools::compile(meshData)) {}

			void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override {
				_shader.setLightPosition({ 7.0f, 5.0f, 2.5f })
					.setTransformationMatrix(transformationMatrix)
					.setNormalMatrix(transformationMatrix.normalMatrix())
					.setProjectionMatrix(camera.projectionMatrix());
				_mesh.draw(_shader);
			}

		private:
			Shaders::Phong& _shader;
			GL::Mesh _mesh;
		};


		TestingApplication::TestingApplication(const Arguments& arguments) : Platform::Application{ arguments, NoCreate } {
			/* Try 8x MSAA, fall back to zero samples if not possible. Enable only 2x
			   MSAA if we have enough DPI. */
			{
				const Vector2 dpiScaling = this->dpiScaling({});
				Configuration conf;
				conf.setTitle("Magnum Mouse Interaction Example")
					.setSize(conf.size(), dpiScaling);
				GLConfiguration glConf;
				glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
				if (!tryCreate(conf, glConf))
					create(conf, glConf.setSampleCount(0));
			}

			/* Shaders, renderer setup */
			_vertexColorShader = Shaders::VertexColor3D{};
			_flatShader = Shaders::Flat3D{};
			_phongShader = Shaders::Phong{};
			GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

			/* Triangle data */
			const struct {
				Vector3 pos;
				Color3 color;
			} data[]{ {{-1.0f, -1.0f, 0.0f}, 0xff0000_rgbf},
					 {{ 1.0f, -1.0f, 0.0f}, 0x00ff00_rgbf},
					 {{ 0.0f,  1.0f, 0.0f}, 0x0000ff_rgbf} };

			/* Triangle mesh */
			GL::Buffer buffer;
			buffer.setData(data);
			_mesh = GL::Mesh{};
			_mesh.setCount(3)
				.addVertexBuffer(std::move(buffer), 0,
					Shaders::VertexColor3D::Position{},
					Shaders::VertexColor3D::Color3{});

			/* Triangle object */
			auto triangle = new Object3D{ &_scene };
			new VertexColorDrawable{ *triangle, _vertexColorShader, _mesh, _drawables };

			/* Grid */
			_grid = MeshTools::compile(Primitives::grid3DWireframe({ 15, 15 }));
			auto grid = new Object3D{ &_scene };
			(*grid)
				.rotateX(90.0_degf)
				.scale(Vector3{ 8.0f });
			new FlatDrawable{ *grid, _flatShader, _grid, _drawables };

			/* Set up the camera */
			_cameraObject = new Object3D{ &_scene };
			(*_cameraObject)
				.translate(Vector3::zAxis(5.0f))
				.rotateX(-15.0_degf)
				.rotateY(30.0_degf);
			_camera = new SceneGraph::Camera3D{ *_cameraObject };
			_camera->setProjectionMatrix(Matrix4::perspectiveProjection(
				45.0_degf, Vector2{ windowSize() }.aspectRatio(), 0.01f, 100.0f));

			/* Initialize initial depth to the value at scene center */
			_lastDepth = ((_camera->projectionMatrix() * _camera->cameraMatrix()).transformPoint({}).z() + 1.0f) * 0.5f;
		}

		Float TestingApplication::depthAt(const Vector2i& windowPosition) {
			/* First scale the position from being relative to window size to being
			   relative to framebuffer size as those two can be different on HiDPI
			   systems */
			const Vector2i position = windowPosition * Vector2{ framebufferSize() } / Vector2{ windowSize() };
			const Vector2i fbPosition{ position.x(), GL::defaultFramebuffer.viewport().sizeY() - position.y() - 1 };

			GL::defaultFramebuffer.mapForRead(GL::DefaultFramebuffer::ReadAttachment::Front);
			Image2D data = GL::defaultFramebuffer.read(
				Range2Di::fromSize(fbPosition, Vector2i{ 1 }).padded(Vector2i{ 2 }),
				{ GL::PixelFormat::DepthComponent, GL::PixelType::Float });

			/* TODO: change to just Math::min<Float>(data.pixels<Float>() when the
			   batch functions in Math can handle 2D views */
			return Math::min<Float>(data.pixels<Float>().asContiguous());
		}

		Vector3 TestingApplication::unproject(const Vector2i& windowPosition, Float depth) const {
			/* We have to take window size, not framebuffer size, since the position is
			   in window coordinates and the two can be different on HiDPI systems */
			const Vector2i viewSize = windowSize();
			const Vector2i viewPosition{ windowPosition.x(), viewSize.y() - windowPosition.y() - 1 };
			const Vector3 in{ 2 * Vector2{viewPosition} / Vector2{viewSize} - Vector2{1.0f}, depth * 2.0f - 1.0f };

			/*
				Use the following to get global coordinates instead of camera-relative:

				(_cameraObject->absoluteTransformationMatrix()*_camera->projectionMatrix().inverted()).transformPoint(in)
			*/
			return _camera->projectionMatrix().inverted().transformPoint(in);
		}

		void TestingApplication::keyPressEvent(KeyEvent& event) {
			/* Reset the transformation to the original view */
			if (event.key() == KeyEvent::Key::NumZero) {
				(*_cameraObject)
					.resetTransformation()
					.translate(Vector3::zAxis(5.0f))
					.rotateX(-15.0_degf)
					.rotateY(30.0_degf);
				redraw();
				return;

				/* Axis-aligned view */
			}
			else if (event.key() == KeyEvent::Key::NumOne ||
				event.key() == KeyEvent::Key::NumThree ||
				event.key() == KeyEvent::Key::NumSeven)
			{
				/* Start with current camera translation with the rotation inverted */
				const Vector3 viewTranslation = _cameraObject->transformation().rotationScaling().inverted() * _cameraObject->transformation().translation();

				/* Front/back */
				const Float multiplier = event.modifiers() & KeyEvent::Modifier::Ctrl ? -1.0f : 1.0f;

				Matrix4 transformation;
				if (event.key() == KeyEvent::Key::NumSeven) /* Top/bottom */
					transformation = Matrix4::rotationX(-90.0_degf * multiplier);
				else if (event.key() == KeyEvent::Key::NumOne) /* Front/back */
					transformation = Matrix4::rotationY(90.0_degf - 90.0_degf * multiplier);
				else if (event.key() == KeyEvent::Key::NumThree) /* Right/left */
					transformation = Matrix4::rotationY(90.0_degf * multiplier);
				else CORRADE_INTERNAL_ASSERT_UNREACHABLE();

				_cameraObject->setTransformation(transformation * Matrix4::translation(viewTranslation));
				redraw();
			}
		}

		void TestingApplication::mousePressEvent(MouseEvent& event) {
			/* Due to compatibility reasons, scroll is also reported as a press event,
			   so filter that out. Could be removed once MouseEvent::Button::Wheel is
			   gone from Magnum. */
			if (event.button() != MouseEvent::Button::Left &&
				event.button() != MouseEvent::Button::Middle)
				return;

			const Float currentDepth = depthAt(event.position());
			const Float depth = currentDepth == 1.0f ? _lastDepth : currentDepth;
			_translationPoint = unproject(event.position(), depth);
			/* Update the rotation point only if we're not zooming against infinite
			   depth or if the original rotation point is not yet initialized */
			if (currentDepth != 1.0f || _rotationPoint.isZero()) {
				_rotationPoint = _translationPoint;
				_lastDepth = depth;
			}
		}

		void TestingApplication::mouseMoveEvent(MouseMoveEvent& event) {
			if (_lastPosition == Vector2i{ -1 }) _lastPosition = event.position();
			const Vector2i delta = event.position() - _lastPosition;
			_lastPosition = event.position();

			if (!event.buttons()) return;

			/* Translate */
			if (event.modifiers() & MouseMoveEvent::Modifier::Shift) {
				const Vector3 p = unproject(event.position(), _lastDepth);
				_cameraObject->translateLocal(_translationPoint - p); /* is Z always 0? */
				_translationPoint = p;

				/* Rotate around rotation point */
			}
			else _cameraObject->transformLocal(
				Matrix4::translation(_rotationPoint) *
				Matrix4::rotationX(-0.01_radf * delta.y()) *
				Matrix4::rotationY(-0.01_radf * delta.x()) *
				Matrix4::translation(-_rotationPoint));

			redraw();
		}

		void TestingApplication::mouseScrollEvent(MouseScrollEvent& event) {
			const Float currentDepth = depthAt(event.position());
			const Float depth = currentDepth == 1.0f ? _lastDepth : currentDepth;
			const Vector3 p = unproject(event.position(), depth);
			/* Update the rotation point only if we're not zooming against infinite
			   depth or if the original rotation point is not yet initialized */
			if (currentDepth != 1.0f || _rotationPoint.isZero()) {
				_rotationPoint = p;
				_lastDepth = depth;
			}

			const Float direction = event.offset().y();
			if (!direction) return;

			/* Move towards/backwards the rotation point in cam coords */
			_cameraObject->translateLocal(_rotationPoint * direction * 0.1f);

			event.setAccepted();
			redraw();
		}

		void TestingApplication::drawEvent() {
			GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

			_camera->draw(_drawables);

			swapBuffers();
		}
		void TestingApplication::renderActors(PxRigidActor** actors, const PxU32 numActors, bool shadows, const PxVec3& color, bool changeColorForSleepingActors, bool wireframePass) {
			for (PxU32 i = 0; i < numActors; i++) {
				const PxU32 nbShapes = actors[i]->getNbShapes();
				std::vector<PxShape*> shapes(nbShapes);
				actors[i]->getShapes(shapes.data(), nbShapes);

				bool sleeping = changeColorForSleepingActors && actors[i]->is<PxRigidDynamic>() ? actors[i]->is<PxRigidDynamic>()->isSleeping() : false;
				const bool isDynamic = actors[i]->getType() == PxActorType::eRIGID_DYNAMIC;

				for (PxU32 j = 0; j < nbShapes; j++) {
					PxVec3 randomColor = color;
					if (isDynamic) {
						srand(std::hash<PxShape*>()(shapes[j]));
						float randomFloat0 = float(rand()) / RAND_MAX;
						float randomFloat1 = float(rand()) / RAND_MAX;
						float randomFloat2 = float(rand()) / RAND_MAX;
						randomColor = PxVec3(randomFloat0, randomFloat1, randomFloat2) * 0.7;
					}

					const PxTransform shapePose = PxShapeExt::getGlobalPose(*shapes[j], *actors[i]);
					Matrix4 transformation = Matrix4::translation(Vector3(shapePose.p.x, shapePose.p.y, shapePose.p.z)) *
						Matrix4::rotation(Rad(shapePose.q.w), Vector3(shapePose.q.x, shapePose.q.y, shapePose.q.z));

					const PxGeometry& geom = shapes[j]->getGeometry();

					// 根据几何类型创建相应的网格
					Trade::MeshData meshData = Primitives::cubeSolid(); // 默认值
					if (geom.getType() == PxGeometryType::eBOX) {
						meshData = Primitives::cubeSolid();
					}
					else if (geom.getType() == PxGeometryType::eSPHERE) {
						meshData = Primitives::uvSphereSolid(16, 32); // 需要提供分段数
					}
					else if (geom.getType() == PxGeometryType::eCAPSULE) {
						meshData = Primitives::cylinderSolid(16, 32, 1.0f); // 需要提供分段数和高度
					}
					else if (geom.getType() == PxGeometryType::ePLANE) {
						meshData = Primitives::planeSolid();
					}
					else {
						continue; // 其他类型暂不处理
					}

					auto* object = new Object3D{ &_scene };
					object->setTransformation(transformation);
					new RenderableObject{ *object, _phongShader, meshData, _drawables };
				}
			}

			// 处理线框模式
			if (wireframePass) {
				GL::Renderer::setPolygonMode(GL::Renderer::PolygonMode::Line);
				// 这里可以重复上面的渲染逻辑，只是将颜色设置为黑色
				GL::Renderer::setPolygonMode(GL::Renderer::PolygonMode::Fill);
			}
		}
}

PhysicsEngineTestingApplication* CreatePhysicsEngineTestingApplication(int argc, char** argv)
{
	return new Magnum::TestingApplication({ argc, argv });
}
