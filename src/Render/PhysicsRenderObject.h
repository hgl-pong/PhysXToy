#pragma once
#include "MagnumRender/MagnumRenderCommon.h"
#include "MagnumRender/RenderUnit.h"
#include "Physics/PhysicsCommon.h"
#include "MeshDataLoader.h"
namespace MagnumRender
{
	class PhysicsRenderObject : public RenderObject {
	public:
		explicit PhysicsRenderObject(const PhysicsPtr<IPhysicsObject>& physicsObject)
			: m_PhysicsObject(physicsObject)
		{
			std::vector<PhysicsPtr<IColliderGeometry>> geometries;
			std::vector<MathLib::HTransform3> transforms;
			m_bIsDynamic = physicsObject->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
			physicsObject->GetColliderGeometries(geometries,&transforms);
			for (size_t i = 0; i < geometries.size(); i++)
			{
				const auto& geometry = geometries[i];
				CollisionGeometryCreateOptions options;
				geometry->GetParams(options);

				Magnum::Trade::MeshData meshData = Magnum::Primitives::cubeSolid();
				Magnum::Matrix4 matrix;
				if (!transforms.empty())
				{
					MathLib::HMatrix4 transposeMatrix = transforms[i].matrix().transpose();
					matrix = ToMagnum(transposeMatrix);
				}
				Magnum::Matrix4 scalingMatrix= Magnum::Matrix4::scaling(Magnum::Vector3(1.f));
				Magnum::Matrix4x4 rotateMatrix;
				PhysicsMeshData newMeshData;
				switch (options.m_GeometryType)
				{
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
				{
					meshData = CreateMesh(MathLib::GraphicUtils::GenerateSphereMeshData(options.m_SphereParams.m_Radius, 8, 8));
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				}
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
				{
					meshData = CreateMesh(MathLib::GraphicUtils::GenerateBoxMeshData(options.m_BoxParams.m_HalfExtents));
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				}
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
				{
					meshData = CreateMesh(MathLib::GraphicUtils::GenerateCapsuleMeshData(options.m_CapsuleParams.m_Radius, options.m_CapsuleParams.m_HalfHeight, 8, 8));
					scalingMatrix = Magnum::Matrix4::scaling(ToMagnum(options.m_Scale));
					break;
				}
				case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
					meshData = Magnum::Primitives::planeSolid();
					rotateMatrix=Magnum::Matrix4::rotationX(90.0_degf);
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
				matrix = matrix * rotateMatrix * scalingMatrix;
				std::shared_ptr<SimpleRenderUnit> newObject = std::make_shared<SimpleRenderUnit>(meshData);
				newObject->SetTransformation(FromMagnum(matrix));
				if(!m_bIsDynamic)
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
				Magnum::Trade::MeshData meshData = CreateMesh(MathLib::GraphicUtils::GenerateBoxWireFrameMeshData(MathLib::HVector3(1,1,1)));
				m_BoundingBox = std::make_shared<GizmoRenderUnit>(meshData);
				m_BoundingBox->SetColor(0x999999_rgbf);
				m_BoundingBox->SetTransformation(&halfSize);
			}
		}

		void UpdateTransform() {
			if (m_Scene == nullptr || m_bShow == false)
				return;
			if ((m_PhysicsObject == nullptr && m_PhysicsObject->GetType() == PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC))
				return;
			const MathLib::HMatrix4& matrix = m_PhysicsObject->GetTransform().matrix();
			MathLib::HMatrix4 transposeMatrix = matrix.transpose();

			m_Object.setTransformation(ToMagnum(transposeMatrix));
			if(m_bShowBoundingBox&&m_UseWorldBoundingBox)
			{
				MathLib::HVector3 halfSize = m_PhysicsObject->GetWorldBoundingBox().sizes() / 2.f;
				MathLib::HVector3 center = m_PhysicsObject->GetWorldBoundingBox().center();
				m_BoundingBox->SetTransformation(&halfSize, &center);
			}
			IDynamicObject* dynamicObject = dynamic_cast<IDynamicObject*>(m_PhysicsObject.get());
			bool isSleeping = dynamicObject ? dynamicObject->IsSleeping() : false;
			for (auto& renderObject : m_RenderObjects)
			{
				if (isSleeping!= m_bIsSleeping)
				{
					Magnum::Color4 ambientColor = renderObject->GetAmbientColor();
					renderObject->SetAmbientColor((isSleeping ? (ambientColor / 2) : ambientColor * 2));
					m_bIsSleeping = isSleeping;
				}
				if(m_bIsDynamic)
					renderObject->UpdateTransformation();
			}
			if(m_bIsDynamic)
				m_BoundingBox->UpdateTransformation();
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
			m_bShow = show;
		}

		void UseWorldBoundingBox(bool useWorldBoundingBox)
		{
			if (m_UseWorldBoundingBox == useWorldBoundingBox)
				return;
			m_UseWorldBoundingBox = useWorldBoundingBox;
			if (m_UseWorldBoundingBox)
			{				
				m_BoundingBox->AddToScene(*m_Scene);
			}
			else
			{
				m_BoundingBox->AddToScene(m_Object);
			}
		}

		MathLib::HAABBox3D GetWorldBoundingBox() const
		{
			if(m_PhysicsObject == nullptr)
				return MathLib::HAABBox3D();
			return m_PhysicsObject->GetWorldBoundingBox();
		}

		void Render(MathLib::GraphicUtils::Camera& camera)
		{
			m_BoundingBox->Render(camera);
			for (auto& renderObject : m_RenderObjects)
			{
				renderObject->Render(camera);
			}
		}

		void AddToScene(Scene3D& scene)
		{
			m_Scene = &scene;
			m_Object.setParent(&scene);	
			UpdateTransform();
			for (auto& renderObject : m_RenderObjects)
			{
				renderObject->AddToScene(m_Object);
			}
			if (m_UseWorldBoundingBox)
			{
				m_BoundingBox->AddToScene(*m_Scene);
			}
			else
			{
				m_BoundingBox->AddToScene(m_Object);
			}

		}

		void RemoveFromScene()
		{
			for (auto& renderObject : m_RenderObjects)
			{
				renderObject->RemoveFromScene();
			}
			m_BoundingBox->RemoveFromScene();
			m_Object.setParent(nullptr);
			m_Scene = nullptr;
		}
	private:
		bool m_bShow= true;
		bool m_bIsSleeping = false;
		bool m_bIsDynamic = false;
		Magnum::SceneGraph::DrawableGroup3D m_RenderCluster;
		Scene3D* m_Scene = nullptr;
		bool m_bShowBoundingBox = true;
		bool m_UseWorldBoundingBox = false;
		std::shared_ptr<GizmoRenderUnit> m_BoundingBox;
		std::vector<std::shared_ptr<SimpleRenderUnit>> m_RenderObjects;
		PhysicsPtr<IPhysicsObject> m_PhysicsObject;
		Object3D m_Object ;
	};
}