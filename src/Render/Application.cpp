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
#include "Camera.h"
using namespace physx;
static MathLib::HVector3 mLightPosition(7.0f, 5.0f, 2.5f);
static Magnum::Color4 mLightColor(0.5f, 0.5f, 0.5f, 1.0f);
namespace Magnum {
		Trade::MeshData CreateMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices)
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
			Containers::Array<char> vertexData{ Containers::NoInit, vertexCount * (sizeof(Vector3) + sizeof(Vector3)) };
			auto positions = Containers::arrayCast<Vector3>(vertexData.prefix(vertexCount * sizeof(Vector3)));
			auto normals = Containers::arrayCast<Vector3>(vertexData.suffix(vertexCount * sizeof(Vector3)));

			for (size_t i = 0; i < vertexCount; ++i)
			{
				new (&positions[i]) Vector3(ToMagnum(vertices[i]));
				new (&normals[i]) Vector3(ToMagnum(normals0[i]));
			}

			Containers::Array<char> indexData{ Containers::NoInit, indices.size() * sizeof(uint32_t) };
			auto indicesArray = Containers::arrayCast<uint32_t>(indexData);
			
			for (size_t i = 0; i < indices.size(); ++i)
				new (&indicesArray[i]) uint32_t(indices[i]);

			return Trade::MeshData{
				MeshPrimitive::Triangles,
				std::move(indexData),
				Trade::MeshIndexData{indicesArray},
				std::move(vertexData),
				{Trade::MeshAttributeData{Trade::MeshAttribute::Position, positions},
					Trade::MeshAttributeData{Trade::MeshAttribute::Normal, normals}} };
		}

		using Object3D = SceneGraph::Object<SceneGraph::MatrixTransformation3D>;
		using Scene3D = SceneGraph::Scene<SceneGraph::MatrixTransformation3D>;

		using namespace Math::Literals;
		class FlatDrawable : public SceneGraph::Drawable3D {
		public:
			explicit FlatDrawable(Object3D& object, Shaders::Flat3D& shader, const Trade::MeshData& meshData, SceneGraph::DrawableGroup3D& drawables) :
				SceneGraph::Drawable3D{ object, &drawables }, _shader(shader), _mesh(MeshTools::compile(meshData)) {}

			void draw(const Matrix4& transformation, SceneGraph::Camera3D& camera) {
				_shader
					.setColor(0x747474_rgbf)
					.setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
					.draw(_mesh);
			}

		private:
			Shaders::Flat3D& _shader;
			GL::Mesh _mesh;
		};

		class RenderableObject : public SceneGraph::Drawable3D {
		public:
			explicit RenderableObject(Object3D& object, Shaders::Phong& shader, const Trade::MeshData& meshData, SceneGraph::DrawableGroup3D& group, const PhysicsPtr<IPhysicsObject>& physicsObject) :
				SceneGraph::Drawable3D{ object, &group }, _shader(shader), _mesh(MeshTools::compile(meshData)), m_PhysicsObject(physicsObject), m_Object(&object) {}

			void UpdateTransform() {
				if ((m_PhysicsObject == nullptr&& m_PhysicsObject->GetType()==PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC) || m_Object == nullptr)
					return;
				const MathLib::HMatrix4& matrix = m_PhysicsObject->GetTransform().matrix();
				MathLib::HMatrix4 transposeMatrix = matrix.transpose();
				MathLib::HMatrix3 R = matrix.block<3, 3>(0, 0);
				MathLib::HVector3 t = matrix.block<3, 1>(0, 3);
				MathLib::HQuaternion q(R);
				const auto transformMatrix= m_Object->transformation();
				const Vector3 scaling = transformMatrix.scaling();
				m_Object->resetTransformation();
				m_Object->scale(scaling);
				m_Object->rotate(ToMagnum(q));
				m_Object->translate(ToMagnum(t));
			}

			void draw(const Matrix4& transformationMatrix, SceneGraph::Camera3D& camera) override {
				_shader.setLightPosition(ToMagnum(mLightPosition))
					.setAmbientColor(m_AmbientColor)
					.setDiffuseColor(m_DiffuseColor)
					.setLightColor(mLightColor)
					.setTransformationMatrix(transformationMatrix)
					.setNormalMatrix(transformationMatrix.normalMatrix())
					.setProjectionMatrix(camera.projectionMatrix());
				_mesh.draw(_shader);
			}

			void SetAmbientColor(const Color4& color) {
				m_AmbientColor = color;
			}
			void SetDiffuseColor(const Color4& color) {
				m_DiffuseColor = color;
			}
		private:
			Color4 m_AmbientColor = { 1.f,1.f,1.f,1.f };
			Color4 m_DiffuseColor = { 1.f,1.f,1.f,1.f };

			Shaders::Phong& _shader;
			GL::Mesh _mesh;
			Object3D* m_Object;
			PhysicsPtr<IPhysicsObject> m_PhysicsObject;
		};
		class TestingApplication : public Platform::Application ,virtual public PhysicsEngineTestingApplication  {
		public:
			explicit TestingApplication(const Arguments& arguments);
		public:
			void Release() override {
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
			Shaders::Flat3D _flatShader{ NoCreate };
			Shaders::Phong _phongShader{ NoCreate };

			Scene3D _scene;
			SceneGraph::DrawableGroup3D _drawables;
			Object3D* _cameraObject;
			SceneGraph::Camera3D* _camera;

			std::vector<RenderableObject*> m_DynamicRenderableObjects;

			PhysicsPtr < IPhysicsMaterial>m_Material;
			PhysicsPtr < IPhysicsScene>m_Scene;
			std::unique_ptr<MathLib::Camera> m_Camera;
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
			m_Camera = std::make_unique<MathLib::Camera>(MathLib::HVector3(50.0f, 50.0f, 50.0f), MathLib::HVector3(-0.6f, -0.6f, -0.6f), MathLib::HReal(Vector2{ framebufferSize()}.aspectRatio()));
			/* Shaders, renderer setup */
			_flatShader = Shaders::Flat3D{};
			_phongShader = Shaders::Phong{};
			GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

			/* Grid */
			auto grid = new Object3D{ &_scene };
			(*grid)
				.rotateX(90.0_degf)
				.scale(Vector3{ 32.0f });
			new FlatDrawable{ *grid, _flatShader, Primitives::grid3DWireframe({ 45, 45 }), _drawables };

			/* Set up the camera */
			_cameraObject = new Object3D{ &_scene };
			_cameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));

			_camera = new SceneGraph::Camera3D{ *_cameraObject };

			_camera->setProjectionMatrix(ToMagnum(m_Camera->getProjectMatrix()));

			_InitPhysics(true);
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
				_cameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));
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

				_CreateDynamic(m_Camera->getTransform(), geometry, m_Camera->getTransform().rotation() * MathLib::HVector3(0, 0, -1) * 100);
				break;
			}
			default:
				break;
			}
			m_Camera->handleAnalogMove(0, 0);
			//_cameraObject->setTransformation(ToMagnumMatrix4(m_Camera->getTransform().matrix()));
			_cameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));

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

			//_cameraObject->setTransformation(ToMagnumMatrix4(m_Camera->getTransform().matrix()));
			_cameraObject->setTransformation(ToMagnum(m_Camera->getViewMatrix()));
			//printf("Dir:%f,%f,%f\n", m_Camera->getDir().x(), m_Camera->getDir().y(), m_Camera->getDir().z());

		}

		void TestingApplication::mouseScrollEvent(MouseScrollEvent& event) {
			event.setAccepted();
		}

		void TestingApplication::drawEvent() {
			GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);
			m_Scene->Tick(1.f/10.f);
			for (auto& renderable : m_DynamicRenderableObjects)
				renderable->UpdateTransform();
			_camera->draw(_drawables);
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
			std::vector<PhysicsPtr<IColliderGeometry>> geometries;
			physicsObject->GetColliderGeometries(geometries);
			for (const auto& geometry : geometries)
			{
				CollisionGeometryCreateOptions options;
				geometry->GetParams(options);
				// 根据几何类型创建相应的网格
				Trade::MeshData meshData = Primitives::cubeSolid();
				auto* object = new Object3D{ &_scene };
				switch (options.m_GeometryType)
				{
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
					meshData = Primitives::uvSphereSolid(16, 32); // 需要提供分段数
					object->scale(ToMagnum(options.m_Scale) * options.m_SphereParams.m_Radius);
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
					meshData = Primitives::cubeSolid();
					object->scale({options.m_BoxParams.m_HalfExtents[0]*options.m_Scale[0],options.m_BoxParams.m_HalfExtents[1]*options.m_Scale[1],options.m_BoxParams.m_HalfExtents[2]*options.m_Scale[2]});
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
					meshData = Primitives::cylinderSolid(16, 32, 1.0f); // 需要提供分段数和高度
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
					meshData = Primitives::planeSolid();
					object->rotateXLocal(90.0_degf);
					object->scale(ToMagnum(options.m_Scale));
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
					meshData =  CreateMesh(options.m_TriangleMeshParams.m_Vertices, options.m_TriangleMeshParams.m_Indices);
					object->scale(ToMagnum(options.m_Scale));
					break;
				default:
					continue; // 其他类型暂不处理
				}

				const MathLib::HMatrix4& matrix = physicsObject->GetTransform().matrix();
				MathLib::HMatrix4 transposeMatrix = matrix.transpose();
				MathLib::HMatrix3 R = matrix.block<3, 3>(0, 0);
				MathLib::HVector3 t = matrix.block<3, 1>(0, 3);
				MathLib::HQuaternion q(R);
				object->rotate(ToMagnum(q));
				object->translate(ToMagnum(t));
				RenderableObject* newObject = new RenderableObject{ *object, _phongShader, meshData, _drawables , physicsObject };
				newObject->SetAmbientColor(0x00110000_rgbaf);
				newObject->SetDiffuseColor(0x00000000_rgbaf);
				m_DynamicRenderableObjects.push_back(newObject);	
			}
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
