#include "Renderer/RenderUnit.h"
#include <chrono>
#include "Math/GraphicUtils/Camara.h"
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <d3d11.h>
#include <DirectXMath.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include <iostream>

using Microsoft::WRL::ComPtr;

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

extern ComPtr<ID3D11Device> g_d3dDevice;
extern ComPtr<ID3D11DeviceContext> g_d3dDeviceContext;

struct ModelViewProjectionConstantBuffer
{
    DirectX::XMMATRIX world;
    DirectX::XMMATRIX view;
    DirectX::XMMATRIX projection;
    DirectX::XMFLOAT4 lightPos;
    DirectX::XMFLOAT4 viewPos;
    DirectX::XMFLOAT4 ambientColor;
    DirectX::XMFLOAT4 diffuseColor;
    DirectX::XMFLOAT4 specularParams; 
    DirectX::XMFLOAT4 lightParams;    
    int isWireframe;
    DirectX::XMFLOAT3 padding;
};

const char* vertexShaderSource = R"(
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
    matrix World;
    matrix View;
    matrix Projection;
    float4 LightPos;
    float4 ViewPos;
    float4 AmbientColor;
    float4 DiffuseColor;
    float4 SpecularParams;
    float4 LightParams;
    int IsWireframe;
    float3 padding;
};

struct VS_INPUT
{
    float4 Pos : POSITION;
    float3 Normal : NORMAL;
};

struct PS_INPUT
{
    float4 Pos : SV_POSITION;
    float3 FragPos : POSITION;
    float3 Normal : NORMAL;
    float4 Color : COLOR;
};

PS_INPUT main(VS_INPUT input)
{
    PS_INPUT output;
    
    float4 worldPos = mul(World, input.Pos);
    output.FragPos = worldPos.xyz;
    
    output.Normal = mul((float3x3)World, input.Normal);
    
    float4 viewPos = mul(View, worldPos);
    output.Pos = mul(Projection, viewPos);
    
    output.Color = float4(1.0f, 0.0f, 0.0f, 1.0f);
    return output;
}
)";

const char* pixelShaderSource = R"(
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
    matrix World;
    matrix View;
    matrix Projection;
    float4 LightPos;
    float4 ViewPos;
    float4 AmbientColor;
    float4 DiffuseColor;
    float4 SpecularParams;
    float4 LightParams;
    int IsWireframe;
    float3 padding;
};

struct PS_INPUT
{
    float4 Pos : SV_POSITION;
    float3 FragPos : POSITION;
    float3 Normal : NORMAL;
    float4 Color : COLOR;
};

float4 main(PS_INPUT input) : SV_TARGET
{
    if (IsWireframe == 1)
    {
        return DiffuseColor;
    }
    
    float3 norm = normalize(input.Normal);
    float3 lightDir = normalize(LightPos.xyz - input.FragPos);
    
    float diff = max(dot(norm, lightDir), 0.0);
    
    float3 ambient = AmbientColor.rgb;
    
    float3 diffuse = diff * DiffuseColor.rgb;
    
    float3 viewDir = normalize(ViewPos.xyz - input.FragPos);
    float3 reflectDir = reflect(-lightDir, norm);
    float specularStrength = SpecularParams.x;
    float specularShininess = SpecularParams.y;
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), specularShininess);
    float3 specular = specularStrength * spec * float3(1.0, 1.0, 1.0);
    
    float rimFactor = SpecularParams.z;
    float rimThreshold = SpecularParams.w;
    float rim = 1.0 - max(dot(viewDir, norm), 0.0);
    rim = smoothstep(0.4, rimThreshold, rim);
    float3 rimLight = rim * rimFactor * DiffuseColor.rgb;
    
    float distance = length(LightPos.xyz - input.FragPos);
    float constantAtt = LightParams.x;
    float linearAtt = LightParams.y;
    float quadraticAtt = LightParams.z;
    float minLightThreshold = LightParams.w;
    float attenuation = 1.0 / (constantAtt + linearAtt * distance + quadraticAtt * distance * distance);
    
    float3 finalColor = ambient + (diffuse + specular + rimLight) * attenuation;
    
    finalColor = max(finalColor, float3(minLightThreshold, minLightThreshold, minLightThreshold));
    
    return float4(finalColor, 1.0);
}
)";

const char* lineVertexShaderSource = R"(
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
    matrix World;
    matrix View;
    matrix Projection;
    float4 LineColor;
};

struct VS_INPUT
{
    float4 Pos : POSITION;
};

struct PS_INPUT
{
    float4 Pos : SV_POSITION;
};

PS_INPUT main(VS_INPUT input)
{
    PS_INPUT output;
    
    float4 worldPos = mul(World, input.Pos);
    float4 viewPos = mul(View, worldPos);
    output.Pos = mul(Projection, viewPos);
    
    return output;
}
)";

const char* linePixelShaderSource = R"(
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
    matrix World;
    matrix View;
    matrix Projection;
    float4 LineColor;
};

float4 main() : SV_TARGET
{
    return LineColor;
}
)";

HRESULT CompileShaderFromSource(const char* source, const char* entryPoint, const char* shaderModel, ID3DBlob** shaderBlob) {
    UINT flags = D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
    flags |= D3DCOMPILE_DEBUG;
#endif

    ComPtr<ID3DBlob> errorBlob;
    HRESULT hr = D3DCompile(source, strlen(source), nullptr, nullptr, nullptr, entryPoint, shaderModel, flags, 0, shaderBlob, &errorBlob);
    if (FAILED(hr) && errorBlob) {
        std::cerr << "Shader compilation error: " << static_cast<char*>(errorBlob->GetBufferPointer()) << std::endl;
    }
    return hr;
}

struct SimpleRenderUnit::Impl {
    ComPtr<ID3D11Buffer> vertexBuffer;
    ComPtr<ID3D11Buffer> indexBuffer;
    ComPtr<ID3D11Buffer> normalBuffer;
    ComPtr<ID3D11Buffer> constantBuffer;
    ComPtr<ID3D11VertexShader> vertexShader;
    ComPtr<ID3D11PixelShader> pixelShader;
    ComPtr<ID3D11InputLayout> inputLayout;
    ComPtr<ID3D11RasterizerState> rasterizerStateSolid;
    ComPtr<ID3D11RasterizerState> rasterizerStateWireframe;
    
    MathLib::HMatrix4 transform;
    MathLib::HVector3 position{0, 0, 0};
    MathLib::HVector3 scale{1, 1, 1};
    float ambientColor[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    float diffuseColor[4] = {0.8f, 0.8f, 0.8f, 1.0f};
    float specularParams[4] = {0.5f, 32.0f, 0.3f, 0.8f}; 
    float lightParams[4] = {1.0f, 0.09f, 0.032f, 0.1f};  
    bool visible = true;
    bool wireframe = false;
    int indicesCount = 0;
    void* sceneParent = nullptr;
    
    void CreateBuffers(const MathLib::GraphicUtils::MeshData32& meshData) {
        HRESULT hr;
        
        D3D11_BUFFER_DESC vertexBufferDesc = {};
        vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        vertexBufferDesc.ByteWidth = sizeof(MathLib::HVector3) * meshData.m_Vertices.size();
        vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        vertexBufferDesc.CPUAccessFlags = 0;
        
        D3D11_SUBRESOURCE_DATA vertexData = {};
        vertexData.pSysMem = meshData.m_Vertices.data();
        
        hr = g_d3dDevice->CreateBuffer(&vertexBufferDesc, &vertexData, &vertexBuffer);
        if (FAILED(hr)) {
            std::cerr << "Failed to create vertex buffer" << std::endl;
            return;
        }
        
        D3D11_BUFFER_DESC indexBufferDesc = {};
        indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        indexBufferDesc.ByteWidth = sizeof(uint32_t) * meshData.m_Indices.size();
        indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
        indexBufferDesc.CPUAccessFlags = 0;
        
        D3D11_SUBRESOURCE_DATA indexData = {};
        indexData.pSysMem = meshData.m_Indices.data();
        
        hr = g_d3dDevice->CreateBuffer(&indexBufferDesc, &indexData, &indexBuffer);
        if (FAILED(hr)) {
            std::cerr << "Failed to create index buffer" << std::endl;
            return;
        }
        
        indicesCount = static_cast<int>(meshData.m_Indices.size());
        
        std::vector<MathLib::HVector3> normals(meshData.m_Vertices.size());
        
        for (size_t i = 0; i < normals.size(); i++) {
            normals[i] = MathLib::HVector3(0.0f, 0.0f, 0.0f);
        }
        
        for (size_t i = 0; i < meshData.m_Indices.size(); i += 3) {
            if (i + 2 < meshData.m_Indices.size()) {
                uint32_t idx0 = meshData.m_Indices[i];
                uint32_t idx1 = meshData.m_Indices[i + 1];
                uint32_t idx2 = meshData.m_Indices[i + 2];
                
                if (idx0 < meshData.m_Vertices.size() && 
                    idx1 < meshData.m_Vertices.size() && 
                    idx2 < meshData.m_Vertices.size()) {
                    
                    MathLib::HVector3 edge1 = meshData.m_Vertices[idx1] - meshData.m_Vertices[idx0];
                    MathLib::HVector3 edge2 = meshData.m_Vertices[idx2] - meshData.m_Vertices[idx0];
                    MathLib::HVector3 faceNormal = edge1.cross(edge2);
                    
                    normals[idx0] += faceNormal;
                    normals[idx1] += faceNormal;
                    normals[idx2] += faceNormal;
                }
            }
        }
        
        for (size_t i = 0; i < normals.size(); i++) {
            if (normals[i].squaredNorm() > 0.000001f) {
                normals[i].normalize();
            } else {
                if (meshData.m_Vertices[i].squaredNorm() > 0.000001f) {
            normals[i] = meshData.m_Vertices[i].normalized();
                } else {
                    normals[i] = MathLib::HVector3(0.0f, 1.0f, 0.0f);
                }
            }
        }
        
        D3D11_BUFFER_DESC normalBufferDesc = {};
        normalBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        normalBufferDesc.ByteWidth = sizeof(MathLib::HVector3) * normals.size();
        normalBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        normalBufferDesc.CPUAccessFlags = 0;
        
        D3D11_SUBRESOURCE_DATA normalData = {};
        normalData.pSysMem = normals.data();
        
        hr = g_d3dDevice->CreateBuffer(&normalBufferDesc, &normalData, &normalBuffer);
        if (FAILED(hr)) {
            std::cerr << "Failed to create normal buffer" << std::endl;
            return;
        }
        
        
        D3D11_BUFFER_DESC constantBufferDesc = {};
        constantBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        constantBufferDesc.ByteWidth = sizeof(ModelViewProjectionConstantBuffer);
        constantBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        constantBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        
        hr = g_d3dDevice->CreateBuffer(&constantBufferDesc, nullptr, &constantBuffer);
        if (FAILED(hr)) {
            std::cerr << "Failed to create constant buffer" << std::endl;
            return;
        }
        
        ComPtr<ID3DBlob> vertexShaderBlob;
        hr = CompileShaderFromSource(vertexShaderSource, "main", "vs_5_0", &vertexShaderBlob);
        if (FAILED(hr)) {
            std::cerr << "Failed to compile vertex shader" << std::endl;
            return;
        }
        
        hr = g_d3dDevice->CreateVertexShader(vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(), nullptr, &vertexShader);
        if (FAILED(hr)) {
            std::cerr << "Failed to create vertex shader" << std::endl;
            return;
        }
        
        D3D11_INPUT_ELEMENT_DESC layout[] = {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
            { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 1, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
        };
        
        hr = g_d3dDevice->CreateInputLayout(layout, 2, vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(), &inputLayout);
        if (FAILED(hr)) {
            std::cerr << "Failed to create input layout" << std::endl;
            return;
        }
        
        ComPtr<ID3DBlob> pixelShaderBlob;
        hr = CompileShaderFromSource(pixelShaderSource, "main", "ps_5_0", &pixelShaderBlob);
        if (FAILED(hr)) {
            std::cerr << "Failed to compile pixel shader" << std::endl;
            return;
        }
        
        hr = g_d3dDevice->CreatePixelShader(pixelShaderBlob->GetBufferPointer(), pixelShaderBlob->GetBufferSize(), nullptr, &pixelShader);
        if (FAILED(hr)) {
            std::cerr << "Failed to create pixel shader" << std::endl;
            return;
        }
        
        D3D11_RASTERIZER_DESC rasterizerDesc = {};
        rasterizerDesc.FillMode = D3D11_FILL_SOLID;
        rasterizerDesc.CullMode = D3D11_CULL_NONE;
        rasterizerDesc.FrontCounterClockwise = FALSE;
        rasterizerDesc.DepthClipEnable = TRUE;

        rasterizerDesc.DepthBias = 0;
        rasterizerDesc.DepthBiasClamp = 0.0f;
        rasterizerDesc.SlopeScaledDepthBias = 0.0f;
        
        hr = g_d3dDevice->CreateRasterizerState(&rasterizerDesc, &rasterizerStateSolid);
        if (FAILED(hr)) {
            std::cerr << "Failed to create solid rasterizer state" << std::endl;
            return;
        }
        
        rasterizerDesc.FillMode = D3D11_FILL_WIREFRAME;
        
        hr = g_d3dDevice->CreateRasterizerState(&rasterizerDesc, &rasterizerStateWireframe);
        if (FAILED(hr)) {
            std::cerr << "Failed to create wireframe rasterizer state" << std::endl;
            return;
        }
    }
    
    void CleanUp() {
    }
};

SimpleRenderUnit::SimpleRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData)
    : m_impl(std::make_unique<Impl>())
{
    m_impl->CreateBuffers(meshData);
}

SimpleRenderUnit::~SimpleRenderUnit() {
    m_impl->CleanUp();
}

void SimpleRenderUnit::SetTransformation(const MathLib::HMatrix4* transform) {
    if (transform) {
        MathLib::HMatrix4 scalingMatrix = MathLib::Scaling(m_impl->scale);
        m_impl->transform = scalingMatrix * transform->transpose();
    } else {
        m_impl->transform = MathLib::HMatrix4::Identity();
    }
}

void SimpleRenderUnit::SetTransformation(const MathLib::HVector3* scale, const MathLib::HVector3* position) {
    if (scale) {
        m_impl->scale = *scale;
    }
    
    if (position) {
        m_impl->position = *position;
    }
    
    UpdateTransformation();
}

void SimpleRenderUnit::UpdateTransformation() {
    MathLib::HMatrix4 scaleMat = MathLib::HMatrix4::Identity();
    scaleMat(0, 0) = m_impl->scale.x();
    scaleMat(1, 1) = m_impl->scale.y();
    scaleMat(2, 2) = m_impl->scale.z();
    
    MathLib::HMatrix4 translateMat = MathLib::HMatrix4::Identity();
    translateMat(0, 3) = m_impl->position.x();
    translateMat(1, 3) = m_impl->position.y();
    translateMat(2, 3) = m_impl->position.z();
    
    m_impl->transform = translateMat * scaleMat;
}

void SimpleRenderUnit::SetScale(const MathLib::HVector3& scale) {
    m_impl->scale = scale;
}

void SimpleRenderUnit::Show(bool show) {
    m_impl->visible = show;
}

void SimpleRenderUnit::Render(MathLib::GraphicUtils::Camera& camera) {
    if (!m_impl->visible) {
        return;
    }
    
    D3D11_MAPPED_SUBRESOURCE mappedResource;
    HRESULT hr = g_d3dDeviceContext->Map(m_impl->constantBuffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
    if (SUCCEEDED(hr)) {
        ModelViewProjectionConstantBuffer* dataPtr = reinterpret_cast<ModelViewProjectionConstantBuffer*>(mappedResource.pData);
        
        DirectX::XMMATRIX worldMatrix = DirectX::XMMatrixSet(
            m_impl->transform(0, 0), m_impl->transform(0, 1), m_impl->transform(0, 2), m_impl->transform(0, 3),
            m_impl->transform(1, 0), m_impl->transform(1, 1), m_impl->transform(1, 2), m_impl->transform(1, 3),
            m_impl->transform(2, 0), m_impl->transform(2, 1), m_impl->transform(2, 2), m_impl->transform(2, 3),
            m_impl->transform(3, 0), m_impl->transform(3, 1), m_impl->transform(3, 2), m_impl->transform(3, 3)
        );

        MathLib::HMatrix4 viewMat = camera.GetViewMatrix();
        DirectX::XMMATRIX viewMatrix = DirectX::XMMatrixSet(
            viewMat(0, 0), viewMat(0, 1), viewMat(0, 2), viewMat(0, 3),
            viewMat(1, 0), viewMat(1, 1), viewMat(1, 2), viewMat(1, 3),
            viewMat(2, 0), viewMat(2, 1), viewMat(2, 2), viewMat(2, 3),
            viewMat(3, 0), viewMat(3, 1), viewMat(3, 2), viewMat(3, 3)
        );
        
        MathLib::HMatrix4 projMat = camera.GetProjectMatrix();
        DirectX::XMMATRIX projMatrix = DirectX::XMMatrixSet(
            projMat(0, 0), projMat(0, 1), projMat(0, 2), projMat(0, 3),
            projMat(1, 0), projMat(1, 1), projMat(1, 2), projMat(1, 3),
            projMat(2, 0), projMat(2, 1), projMat(2, 2), projMat(2, 3),
            projMat(3, 0), projMat(3, 1), projMat(3, 2), projMat(3, 3)
        );
        
        MathLib::HVector3 camPos = camera.GetEye();
        
        dataPtr->world = worldMatrix;
        dataPtr->view = viewMatrix;
        dataPtr->projection = projMatrix;
        dataPtr->lightPos = DirectX::XMFLOAT4(50.0f, 50.0f, 50.0f, 1.0f);
        dataPtr->viewPos = DirectX::XMFLOAT4(camPos.x(), camPos.y(), camPos.z(), 1.0f);
        dataPtr->ambientColor = DirectX::XMFLOAT4(m_impl->ambientColor[0], m_impl->ambientColor[1], m_impl->ambientColor[2], m_impl->ambientColor[3]);
        dataPtr->diffuseColor = DirectX::XMFLOAT4(m_impl->diffuseColor[0], m_impl->diffuseColor[1], m_impl->diffuseColor[2], m_impl->diffuseColor[3]);
        dataPtr->specularParams = DirectX::XMFLOAT4(m_impl->specularParams[0], m_impl->specularParams[1], m_impl->specularParams[2], m_impl->specularParams[3]);
        dataPtr->lightParams = DirectX::XMFLOAT4(m_impl->lightParams[0], m_impl->lightParams[1], m_impl->lightParams[2], m_impl->lightParams[3]);
        dataPtr->isWireframe = m_impl->wireframe ? 1 : 0;
        
        g_d3dDeviceContext->Unmap(m_impl->constantBuffer.Get(), 0);
    }
    
    UINT stride = sizeof(MathLib::HVector3);
    UINT offset = 0;
    g_d3dDeviceContext->IASetVertexBuffers(0, 1, m_impl->vertexBuffer.GetAddressOf(), &stride, &offset);
    
    g_d3dDeviceContext->IASetVertexBuffers(1, 1, m_impl->normalBuffer.GetAddressOf(), &stride, &offset);
    
    g_d3dDeviceContext->IASetIndexBuffer(m_impl->indexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
    
    g_d3dDeviceContext->IASetInputLayout(m_impl->inputLayout.Get());
    g_d3dDeviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    
    g_d3dDeviceContext->VSSetShader(m_impl->vertexShader.Get(), nullptr, 0);
    g_d3dDeviceContext->VSSetConstantBuffers(0, 1, m_impl->constantBuffer.GetAddressOf());
    g_d3dDeviceContext->PSSetShader(m_impl->pixelShader.Get(), nullptr, 0);
    g_d3dDeviceContext->PSSetConstantBuffers(0, 1, m_impl->constantBuffer.GetAddressOf());
    
    g_d3dDeviceContext->RSSetState(m_impl->wireframe ? m_impl->rasterizerStateWireframe.Get() : m_impl->rasterizerStateSolid.Get());
    
    g_d3dDeviceContext->DrawIndexed(m_impl->indicesCount, 0, 0);
    
}

void SimpleRenderUnit::AddToScene(void* scene) {
    m_impl->sceneParent = scene;
}

void SimpleRenderUnit::RemoveFromScene() {
    m_impl->sceneParent = nullptr;
}

void SimpleRenderUnit::ShowWireframe(bool show) {
    m_impl->wireframe = show;
}

void SimpleRenderUnit::SetAmbientColor(const float* color) {
    if (color) {
        memcpy(m_impl->ambientColor, color, 4 * sizeof(float));
    }
}

void SimpleRenderUnit::SetDiffuseColor(const float* color) {
    if (color) {
        memcpy(m_impl->diffuseColor, color, 4 * sizeof(float));
    }
}

const float* SimpleRenderUnit::GetAmbientColor() const {
    return m_impl->ambientColor;
}

void SimpleRenderUnit::SetSpecularParams(const float* params) {
    if (params) {
        memcpy(m_impl->specularParams, params, 4 * sizeof(float));
    }
}

const float* SimpleRenderUnit::GetSpecularParams() const {
    return m_impl->specularParams;
}

void SimpleRenderUnit::SetLightParams(const float* params) {
    if (params) {
        memcpy(m_impl->lightParams, params, 4 * sizeof(float));
    }
}

const float* SimpleRenderUnit::GetLightParams() const {
    return m_impl->lightParams;
}

struct GizmoRenderUnit::Impl {
    ComPtr<ID3D11Buffer> vertexBuffer;
    ComPtr<ID3D11Buffer> indexBuffer;
    ComPtr<ID3D11Buffer> constantBuffer;
    ComPtr<ID3D11VertexShader> vertexShader;
    ComPtr<ID3D11PixelShader> pixelShader;
    ComPtr<ID3D11InputLayout> inputLayout;
    ComPtr<ID3D11RasterizerState> rasterizerState;
    
    MathLib::HMatrix4 transform;
    MathLib::HVector3 position{0, 0, 0};
    MathLib::HVector3 scale{1, 1, 1};
    float color[4] = {0.9f, 0.9f, 0.9f, 1.0f};
    bool visible = true;
    int indicesCount = 0;
    void* sceneParent = nullptr;
    
    void CreateBuffers(const MathLib::GraphicUtils::MeshData32& meshData) {
        HRESULT hr;
        
        D3D11_BUFFER_DESC vertexBufferDesc = {};
        vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        vertexBufferDesc.ByteWidth = sizeof(MathLib::HVector3) * meshData.m_Vertices.size();
        vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        vertexBufferDesc.CPUAccessFlags = 0;
        
        D3D11_SUBRESOURCE_DATA vertexData = {};
        vertexData.pSysMem = meshData.m_Vertices.data();
        
        hr = g_d3dDevice->CreateBuffer(&vertexBufferDesc, &vertexData, &vertexBuffer);
        if (FAILED(hr)) {
            std::cerr << "Failed to create vertex buffer for gizmo" << std::endl;
            return;
        }
        
        D3D11_BUFFER_DESC indexBufferDesc = {};
        indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        indexBufferDesc.ByteWidth = sizeof(uint32_t) * meshData.m_Indices.size();
        indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
        indexBufferDesc.CPUAccessFlags = 0;
        
        D3D11_SUBRESOURCE_DATA indexData = {};
        indexData.pSysMem = meshData.m_Indices.data();
        
        hr = g_d3dDevice->CreateBuffer(&indexBufferDesc, &indexData, &indexBuffer);
        if (FAILED(hr)) {
            std::cerr << "Failed to create index buffer for gizmo" << std::endl;
            return;
        }
        
        indicesCount = static_cast<int>(meshData.m_Indices.size());
        
        struct LineConstantBuffer {
            DirectX::XMMATRIX world;
            DirectX::XMMATRIX view;
            DirectX::XMMATRIX projection;
            DirectX::XMFLOAT4 lineColor;
        };
        
        D3D11_BUFFER_DESC constantBufferDesc = {};
        constantBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        constantBufferDesc.ByteWidth = sizeof(LineConstantBuffer);
        constantBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        constantBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        
        hr = g_d3dDevice->CreateBuffer(&constantBufferDesc, nullptr, &constantBuffer);
        if (FAILED(hr)) {
            std::cerr << "Failed to create constant buffer for gizmo" << std::endl;
            return;
        }
        
        ComPtr<ID3DBlob> vertexShaderBlob;
        hr = CompileShaderFromSource(lineVertexShaderSource, "main", "vs_5_0", &vertexShaderBlob);
        if (FAILED(hr)) {
            std::cerr << "Failed to compile line vertex shader" << std::endl;
            return;
        }
        
        hr = g_d3dDevice->CreateVertexShader(vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(), nullptr, &vertexShader);
        if (FAILED(hr)) {
            std::cerr << "Failed to create line vertex shader" << std::endl;
            return;
        }
        
        D3D11_INPUT_ELEMENT_DESC layout[] = {
            { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
        };
        
        hr = g_d3dDevice->CreateInputLayout(layout, 1, vertexShaderBlob->GetBufferPointer(), vertexShaderBlob->GetBufferSize(), &inputLayout);
        if (FAILED(hr)) {
            std::cerr << "Failed to create input layout for gizmo" << std::endl;
            return;
        }
        
        ComPtr<ID3DBlob> pixelShaderBlob;
        hr = CompileShaderFromSource(linePixelShaderSource, "main", "ps_5_0", &pixelShaderBlob);
        if (FAILED(hr)) {
            std::cerr << "Failed to compile line pixel shader" << std::endl;
            return;
        }
        
        hr = g_d3dDevice->CreatePixelShader(pixelShaderBlob->GetBufferPointer(), pixelShaderBlob->GetBufferSize(), nullptr, &pixelShader);
        if (FAILED(hr)) {
            std::cerr << "Failed to create line pixel shader" << std::endl;
            return;
        }
        
        D3D11_RASTERIZER_DESC rasterizerDesc = {};
        rasterizerDesc.FillMode = D3D11_FILL_WIREFRAME;
        rasterizerDesc.CullMode = D3D11_CULL_NONE;
        rasterizerDesc.FrontCounterClockwise = FALSE;
        rasterizerDesc.DepthClipEnable = TRUE;
        
        hr = g_d3dDevice->CreateRasterizerState(&rasterizerDesc, &rasterizerState);
        if (FAILED(hr)) {
            std::cerr << "Failed to create rasterizer state for gizmo" << std::endl;
            return;
        }
    }
    
    void CleanUp() {
    }
};

GizmoRenderUnit::GizmoRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData)
    : m_impl(std::make_unique<Impl>())
{
    m_impl->CreateBuffers(meshData);
}

GizmoRenderUnit::~GizmoRenderUnit() {
    m_impl->CleanUp();
}

void GizmoRenderUnit::SetTransformation(const MathLib::HMatrix4* transform) {
    if (transform) {
        m_impl->transform = transform->transpose();
    } else {
        m_impl->transform = MathLib::HMatrix4::Identity();
    }
}

void GizmoRenderUnit::SetTransformation(const MathLib::HVector3* scale, const MathLib::HVector3* position) {
    if (scale) {
        m_impl->scale = *scale;
    }
    
    if (position) {
        m_impl->position = *position;
    }
    
    UpdateTransformation();
}

void GizmoRenderUnit::UpdateTransformation() {
    MathLib::HMatrix4 scaleMat = MathLib::HMatrix4::Identity();
    scaleMat(0, 0) = m_impl->scale.x();
    scaleMat(1, 1) = m_impl->scale.y();
    scaleMat(2, 2) = m_impl->scale.z();
    
    MathLib::HMatrix4 translateMat = MathLib::HMatrix4::Identity();
    translateMat(0, 3) = m_impl->position.x();
    translateMat(1, 3) = m_impl->position.y();
    translateMat(2, 3) = m_impl->position.z();
    
    m_impl->transform = translateMat * scaleMat;
}

void GizmoRenderUnit::Show(bool show) {
    m_impl->visible = show;
}

void GizmoRenderUnit::Render(MathLib::GraphicUtils::Camera& camera) {
    if (!m_impl->visible) {
        return;
    }
    
    D3D11_MAPPED_SUBRESOURCE mappedResource;
    HRESULT hr = g_d3dDeviceContext->Map(m_impl->constantBuffer.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
    if (SUCCEEDED(hr)) {
        struct LineConstantBuffer {
            DirectX::XMMATRIX world;
            DirectX::XMMATRIX view;
            DirectX::XMMATRIX projection;
            DirectX::XMFLOAT4 lineColor;
        };
        
        LineConstantBuffer* dataPtr = reinterpret_cast<LineConstantBuffer*>(mappedResource.pData);
        
        DirectX::XMMATRIX worldMatrix = DirectX::XMMatrixSet(
            m_impl->transform(0, 0), m_impl->transform(0, 1), m_impl->transform(0, 2), m_impl->transform(0, 3),
            m_impl->transform(1, 0), m_impl->transform(1, 1), m_impl->transform(1, 2), m_impl->transform(1, 3),
            m_impl->transform(2, 0), m_impl->transform(2, 1), m_impl->transform(2, 2), m_impl->transform(2, 3),
            m_impl->transform(3, 0), m_impl->transform(3, 1), m_impl->transform(3, 2), m_impl->transform(3, 3)
        );
        
        MathLib::HMatrix4 viewMat = camera.GetViewMatrix();
        DirectX::XMMATRIX viewMatrix = DirectX::XMMatrixSet(
            viewMat(0, 0), viewMat(0, 1), viewMat(0, 2), viewMat(0, 3),
            viewMat(1, 0), viewMat(1, 1), viewMat(1, 2), viewMat(1, 3),
            viewMat(2, 0), viewMat(2, 1), viewMat(2, 2), viewMat(2, 3),
            viewMat(3, 0), viewMat(3, 1), viewMat(3, 2), viewMat(3, 3)
        );
        
        MathLib::HMatrix4 projMat = camera.GetProjectMatrix();
        DirectX::XMMATRIX projMatrix = DirectX::XMMatrixSet(
            projMat(0, 0), projMat(0, 1), projMat(0, 2), projMat(0, 3),
            projMat(1, 0), projMat(1, 1), projMat(1, 2), projMat(1, 3),
            projMat(2, 0), projMat(2, 1), projMat(2, 2), projMat(2, 3),
            projMat(3, 0), projMat(3, 1), projMat(3, 2), projMat(3, 3)
        );
        
        dataPtr->world = worldMatrix;
        dataPtr->view = viewMatrix;
        dataPtr->projection = projMatrix;
        dataPtr->lineColor = DirectX::XMFLOAT4(m_impl->color[0], m_impl->color[1], m_impl->color[2], m_impl->color[3]);
        
        g_d3dDeviceContext->Unmap(m_impl->constantBuffer.Get(), 0);
    }
    
    UINT stride = sizeof(MathLib::HVector3);
    UINT offset = 0;
    g_d3dDeviceContext->IASetVertexBuffers(0, 1, m_impl->vertexBuffer.GetAddressOf(), &stride, &offset);
    g_d3dDeviceContext->IASetIndexBuffer(m_impl->indexBuffer.Get(), DXGI_FORMAT_R32_UINT, 0);
    g_d3dDeviceContext->IASetInputLayout(m_impl->inputLayout.Get());
    
    g_d3dDeviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
    
    g_d3dDeviceContext->VSSetShader(m_impl->vertexShader.Get(), nullptr, 0);
    g_d3dDeviceContext->VSSetConstantBuffers(0, 1, m_impl->constantBuffer.GetAddressOf());
    g_d3dDeviceContext->PSSetShader(m_impl->pixelShader.Get(), nullptr, 0);
    g_d3dDeviceContext->PSSetConstantBuffers(0, 1, m_impl->constantBuffer.GetAddressOf());
    
    g_d3dDeviceContext->RSSetState(m_impl->rasterizerState.Get());
    
    int actualIndicesCount = m_impl->indicesCount;
    if (actualIndicesCount % 2 != 0) {
        actualIndicesCount--;
    }
    if (actualIndicesCount > 0) {
        g_d3dDeviceContext->DrawIndexed(actualIndicesCount, 0, 0);
    }
}

void GizmoRenderUnit::AddToScene(void* scene) {
    m_impl->sceneParent = scene;
}

void GizmoRenderUnit::RemoveFromScene() {
    m_impl->sceneParent = nullptr;
}

void GizmoRenderUnit::SetColor(const float* color) {
    if (color) {
        memcpy(m_impl->color, color, 4 * sizeof(float));
    }
}