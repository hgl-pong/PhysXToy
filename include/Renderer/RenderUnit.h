#pragma once
#include <memory>
#include <vector>
#include "Renderer/Renderer.h"
#include "Math/GraphicUtils/MeshData.h"


class BaseRenderUnit {
public:
    virtual ~BaseRenderUnit() = default;
    
    virtual void SetTransformation(const MathLib::HMatrix4* transform = nullptr) = 0;
    
    virtual void SetTransformation(const MathLib::HVector3* scale = nullptr, const MathLib::HVector3* position = nullptr) = 0;
    
    virtual void UpdateTransformation() = 0;
    
    virtual void Show(bool show) = 0;

    virtual void Render(MathLib::GraphicUtils::Camera& camera) = 0;
    
    virtual void AddToScene(void* scene) = 0;

    virtual void RemoveFromScene() = 0;
};


class SimpleRenderUnit : public BaseRenderUnit {
public:
    explicit SimpleRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData);
    ~SimpleRenderUnit() override;
    
    void SetScale(const MathLib::HVector3& scale);
    void SetTransformation(const MathLib::HMatrix4* transformation = nullptr) override;
    void SetTransformation(const MathLib::HVector3* scale = nullptr, const MathLib::HVector3* position = nullptr) override;
    void UpdateTransformation() override;
    void Show(bool show) override;
    void Render(MathLib::GraphicUtils::Camera& camera) override;
    void AddToScene(void* scene) override;
    void RemoveFromScene() override;
    
    void ShowWireframe(bool show);
    
    void SetAmbientColor(const float* color);
    
    void SetDiffuseColor(const float* color);
    
    const float* GetAmbientColor() const;
    
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

class GizmoRenderUnit : public BaseRenderUnit {
public:
    explicit GizmoRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData);
    ~GizmoRenderUnit() override;
    
    void SetTransformation(const MathLib::HMatrix4* transform = nullptr) override;
    void SetTransformation(const MathLib::HVector3* scale = nullptr, const MathLib::HVector3* position = nullptr) override;
    void UpdateTransformation() override;
    void Show(bool show) override;
    void Render(MathLib::GraphicUtils::Camera& camera) override;
    void AddToScene(void* scene) override;
    void RemoveFromScene() override;

    void SetColor(const float* color);
    
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
}; 