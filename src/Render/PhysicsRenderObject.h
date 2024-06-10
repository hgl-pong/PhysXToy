#pragma once
#include "MagnumRender/RenderObject.h"
#include "Physics/PhysicsCommon.h"
#include "MeshDataLoader.h"
namespace MagnumRender
{
	class PhysicsRenderObject {
	public:
		explicit PhysicsRenderObject(Scene3D& renderScene, const PhysicsPtr<IPhysicsObject>& physicsObject)
			: m_PhysicsObject(physicsObject)
		{
			m_Object = std::make_shared < Object3D>();
			m_Object->setParent(&renderScene);

			std::vector<PhysicsPtr<IColliderGeometry>> geometries;
			std::vector<MathLib::HTransform3> transforms;
			bool isDynamic = physicsObject->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
			physicsObject->GetColliderGeometries(geometries,&transforms);
			for (size_t i = 0; i < geometries.size(); i++)
			{
				const auto& geometry = geometries[i];
				CollisionGeometryCreateOptions options;
				geometry->GetParams(options);

				Magnum::Trade::MeshData meshData = Magnum::Primitives::cubeSolid();
				auto* object = new Object3D{ &renderScene };
				Magnum::Matrix4 matrix;
				if (!transforms.empty())
				{
					MathLib::HMatrix4 transposeMatrix = transforms[i].matrix().transpose();
					matrix = ToMagnum(transposeMatrix);
				}
				Magnum::Matrix4 scalingMatrix= Magnum::Matrix4::scaling(Magnum::Vector3(1.f));
				PhysicsMeshData newMeshData;
				switch (options.m_GeometryType)
				{
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
				{
					newMeshData = MathLib::GraphicUtils::GenerateSphereMeshData(options.m_SphereParams.m_Radius, 8, 8);
					meshData = CreateMesh(newMeshData.m_Vertices, newMeshData.m_Indices);
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				}
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
				{
					newMeshData = MathLib::GraphicUtils::GenerateBoxMeshData(options.m_BoxParams.m_HalfExtents);
					meshData = CreateMesh(newMeshData.m_Vertices, newMeshData.m_Indices);
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				}
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
				{
					 newMeshData = MathLib::GraphicUtils::GenerateCapsuleMeshData(options.m_CapsuleParams.m_Radius, options.m_CapsuleParams.m_HalfHeight, 8, 8);
					meshData = CreateMesh(newMeshData.m_Vertices, newMeshData.m_Indices);
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				}
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
					meshData = Magnum::Primitives::planeSolid();
					object->rotateXLocal(90.0_degf);
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
					meshData = CreateMesh(options.m_TriangleMeshParams.m_Vertices, options.m_TriangleMeshParams.m_Indices);
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
					meshData = CreateMesh(options.m_ConvexMeshParams.m_Vertices, options.m_ConvexMeshParams.m_Indices);
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				default:
					continue;
				}
				matrix = matrix * scalingMatrix;
				object->setTransformation(matrix);
				object->setParent(m_Object.get());
				std::shared_ptr<RenderUnit> newObject = std::make_shared<RenderUnit>(*object,meshData, m_RenderCluster, matrix);
				if(!isDynamic)
					newObject->SetAmbientColor(0x66000000_rgbaf);
				else
				{
					float randomFloat0 = float(rand()) / RAND_MAX;
					float randomFloat1 = float(rand()) / RAND_MAX;
					float randomFloat2 = float(rand()) / RAND_MAX;
					Magnum::Color4 ambientColor = Magnum::Color4(randomFloat0, randomFloat1, randomFloat2, 1.0f);
					newObject->SetAmbientColor(ambientColor);
				}
				newObject->SetDiffuseColor(0x55555555_rgbaf);
				m_RenderObjects.push_back(newObject);
			}

			{
				MathLib::HVector3 halfSize = physicsObject->GetLocalBoundingBox().sizes() / 2.f;
				//Trade::MeshData meshData = Primitives::cubeWireframe();
				PhysicsMeshData newMeshData = MathLib::GraphicUtils::GenerateBoxWireFrameMeshData(halfSize);
				Magnum::Trade::MeshData meshData = CreateMesh(newMeshData.m_Vertices, newMeshData.m_Indices);
				m_BoundingBoxObject = std::make_shared < Object3D>(&renderScene);
				m_ParentObject = m_BoundingBoxObject->parent();
				m_BoundingBoxObject->setParent(m_Object.get());
				m_BoundingBox = std::make_shared<FlatDrawable>(meshData);
				m_BoundingBox->AddToScene(renderScene);
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
				m_BoundingBoxObject->scale(Magnum::Vector3(halfSize[0], halfSize[1], halfSize[2]));
				auto meshData= Magnum::Primitives::cubeWireframe();
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
				//Trade::MeshData meshData = Primitives::cubeWireframe();
				//m_BoundingBoxObject->scale(Vector3(halfSize[0], halfSize[1], halfSize[2]));
				PhysicsMeshData newMeshData = MathLib::GraphicUtils::GenerateBoxWireFrameMeshData(halfSize);
				Magnum::Trade::MeshData meshData = CreateMesh(newMeshData.m_Vertices, newMeshData.m_Indices);
				m_BoundingBox->SetMeshData(meshData);
				m_BoundingBoxObject->setParent(m_Object.get());
			}
		}

		MathLib::HAABBox3D GetWorldBoundingBox() const
		{
			if(m_PhysicsObject == nullptr)
				return MathLib::HAABBox3D();
			return m_PhysicsObject->GetWorldBoundingBox();
		}

		void Render(Magnum::SceneGraph::Camera3D* camera)
		{
			if (camera == nullptr)
				return;
			camera->draw(m_RenderCluster);
			//m_BoundingBox->Render(*camera);
		}
	private:
		Magnum::SceneGraph::DrawableGroup3D m_RenderCluster;
		Object3D* m_ParentObject = nullptr;
		bool m_bShowBoundingBox = true;
		bool m_UseWorldBoundingBox = false;
		std::shared_ptr<FlatDrawable> m_BoundingBox;
		std::vector<std::shared_ptr<RenderUnit>> m_RenderObjects;
		PhysicsPtr<IPhysicsObject> m_PhysicsObject;
		std::shared_ptr < Object3D> m_BoundingBoxObject ;
		std::shared_ptr < Object3D> m_Object ;
	};
}