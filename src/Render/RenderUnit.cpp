#include "Renderer/RenderUnit.h"
#include <chrono>
#include "Math/GraphicUtils/Camara.h"
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

// 顶点着色器
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 FragPos;
out vec3 Normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;  
    
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

// 片段着色器 - 标准材质
const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec4 objectColor;
uniform vec4 ambientColor;
uniform bool isWireframe;

void main()
{
    if (isWireframe) {
        FragColor = objectColor;
    } else {
        // 环境光
        vec3 ambient = 0.2 * ambientColor.rgb;
        
        // 漫反射
        vec3 norm = normalize(Normal);
        vec3 lightDir = normalize(lightPos - FragPos);
        float diff = max(dot(norm, lightDir), 0.0);
        vec3 diffuse = diff * objectColor.rgb;
        
        // 高光
        vec3 viewDir = normalize(viewPos - FragPos);
        vec3 reflectDir = reflect(-lightDir, norm);  
        float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
        vec3 specular = 0.5 * spec * vec3(1.0, 1.0, 1.0);  
        
        vec3 result = ambient + diffuse + specular;
        FragColor = vec4(result, objectColor.a);
    }
}
)";

// 线框着色器
const char* lineShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

const char* lineFragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

uniform vec4 lineColor;

void main()
{
    FragColor = lineColor;
}
)";

// 编译着色器辅助函数
GLuint compileShader(GLenum shaderType, const char* source) {
    GLuint shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    
    // 检查编译错误
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
        return 0;
    }
    return shader;
}

// 创建着色器程序
GLuint createShaderProgram(const char* vertexSource, const char* fragmentSource) {
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
    
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    
    // 检查链接错误
    GLint success;
    GLchar infoLog[512];
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        return 0;
    }
    
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    return shaderProgram;
}

// SimpleRenderUnit实现
struct SimpleRenderUnit::Impl {
    GLuint VAO, VBO, EBO;
    GLuint shaderProgram;
    MathLib::HMatrix4 transform;
    MathLib::HVector3 position{0, 0, 0};
    MathLib::HVector3 scale{1, 1, 1};
    float ambientColor[4] = {0.5f, 0.5f, 0.5f, 1.0f};
    float diffuseColor[4] = {0.8f, 0.8f, 0.8f, 1.0f};
    bool visible = true;
    bool wireframe = false;
    int indicesCount = 0;
    void* sceneParent = nullptr;
    
    void CreateBuffers(const MathLib::GraphicUtils::MeshData32& meshData) {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        
        glBindVertexArray(VAO);
        
        // 顶点数据
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, meshData.m_Vertices.size() * sizeof(MathLib::HVector3), meshData.m_Vertices.data(), GL_STATIC_DRAW);
        
        // 索引数据
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshData.m_Indices.size() * sizeof(uint32_t), meshData.m_Indices.data(), GL_STATIC_DRAW);
        
        // 设置顶点属性指针
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MathLib::HVector3), (void*)0);
        glEnableVertexAttribArray(0);
        
        // 简单的法线计算（假设每个顶点法线指向顶点位置方向）
        std::vector<MathLib::HVector3> normals(meshData.m_Vertices.size());
        for (size_t i = 0; i < meshData.m_Vertices.size(); i++) {
            normals[i] = meshData.m_Vertices[i].normalized();
        }
        
        // 创建并绑定法线缓冲区
        GLuint normalVBO;
        glGenBuffers(1, &normalVBO);
        glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
        glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(MathLib::HVector3), normals.data(), GL_STATIC_DRAW);
        
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MathLib::HVector3), (void*)0);
        glEnableVertexAttribArray(1);
        
        indicesCount = static_cast<int>(meshData.m_Indices.size());
        
        // 解绑
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        
        // 创建着色器
        shaderProgram = createShaderProgram(vertexShaderSource, fragmentShaderSource);
    }
    
    void CleanUp() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glDeleteProgram(shaderProgram);
    }
};

SimpleRenderUnit::SimpleRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData)
    : m_impl(new Impl()) {
    m_impl->CreateBuffers(meshData);
}

SimpleRenderUnit::~SimpleRenderUnit() {
    m_impl->CleanUp();
}

void SimpleRenderUnit::SetTransformation(const MathLib::HMatrix4* transform) {
    if (transform) {
        m_impl->transform = *transform;
    }
}

void SimpleRenderUnit::SetTransformation(const MathLib::HVector3* scale, const MathLib::HVector3* position) {
    if (scale) {
        m_impl->scale = *scale;
    }
    if (position) {
        m_impl->position = *position;
    }
    
    // 构建变换矩阵
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(m_impl->position[0], m_impl->position[1], m_impl->position[2]));
    model = glm::scale(model, glm::vec3(m_impl->scale[0], m_impl->scale[1], m_impl->scale[2]));
    
    m_impl->transform = *reinterpret_cast<MathLib::HMatrix4*>(&model);
}

void SimpleRenderUnit::UpdateTransformation() {
    // 如果需要动态更新变换矩阵，在这里实现
}

void SimpleRenderUnit::Show(bool show) {
    m_impl->visible = show;
}

void SimpleRenderUnit::Render(MathLib::GraphicUtils::Camera& camera) {
    if (!m_impl->visible) return;
    
    glUseProgram(m_impl->shaderProgram);
    
    // 设置着色器参数
    glUniformMatrix4fv(glGetUniformLocation(m_impl->shaderProgram, "model"), 1, GL_FALSE, m_impl->transform.data());
    
    // 假设Camera类提供了视图和投影矩阵
    MathLib::HMatrix4 viewMatrix = camera.GetViewMatrix();
    MathLib::HMatrix4 projMatrix = camera.GetProjectMatrix();
    
    glUniformMatrix4fv(glGetUniformLocation(m_impl->shaderProgram, "view"), 1, GL_FALSE, viewMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(m_impl->shaderProgram, "projection"), 1, GL_FALSE, projMatrix.data());
    
    // 设置光源位置 (可以是摄像机位置)
    MathLib::HVector3 cameraPos = camera.GetEye();
    glUniform3f(glGetUniformLocation(m_impl->shaderProgram, "lightPos"), cameraPos[0], cameraPos[1], cameraPos[2]);
    glUniform3f(glGetUniformLocation(m_impl->shaderProgram, "viewPos"), cameraPos[0], cameraPos[1], cameraPos[2]);
    
    // 设置颜色
    glUniform4fv(glGetUniformLocation(m_impl->shaderProgram, "objectColor"), 1, m_impl->diffuseColor);
    glUniform4fv(glGetUniformLocation(m_impl->shaderProgram, "ambientColor"), 1, m_impl->ambientColor);
    glUniform1i(glGetUniformLocation(m_impl->shaderProgram, "isWireframe"), m_impl->wireframe ? 1 : 0);
    
    // 绘制
    glBindVertexArray(m_impl->VAO);
    if (m_impl->wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    
    glDrawElements(GL_TRIANGLES, m_impl->indicesCount, GL_UNSIGNED_INT, 0);
    
    // 恢复默认设置
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindVertexArray(0);
}

void SimpleRenderUnit::AddToScene(void* scene) {
    m_impl->sceneParent = scene;
    // 可能需要更多场景整合代码
}

void SimpleRenderUnit::RemoveFromScene() {
    m_impl->sceneParent = nullptr;
}

void SimpleRenderUnit::ShowWireframe(bool show) {
    m_impl->wireframe = show;
}

void SimpleRenderUnit::SetAmbientColor(const float* color) {
    if (color) {
        for (int i = 0; i < 4; i++) {
            m_impl->ambientColor[i] = color[i];
        }
    }
}

void SimpleRenderUnit::SetDiffuseColor(const float* color) {
    if (color) {
        for (int i = 0; i < 4; i++) {
            m_impl->diffuseColor[i] = color[i];
        }
    }
}

const float* SimpleRenderUnit::GetAmbientColor() const {
    return m_impl->ambientColor;
}

// GizmoRenderUnit实现
struct GizmoRenderUnit::Impl {
    GLuint VAO, VBO, EBO;
    GLuint shaderProgram;
    MathLib::HMatrix4 transform;
    MathLib::HVector3 position{0, 0, 0};
    MathLib::HVector3 scale{1, 1, 1};
    float color[4] = {0.9f, 0.9f, 0.9f, 1.0f};
    bool visible = true;
    int indicesCount = 0;
    void* sceneParent = nullptr;
    
    void CreateBuffers(const MathLib::GraphicUtils::MeshData32& meshData) {
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        
        glBindVertexArray(VAO);
        
        // 顶点数据
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, meshData.m_Vertices.size() * sizeof(MathLib::HVector3), meshData.m_Vertices.data(), GL_STATIC_DRAW);
        
        // 索引数据
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshData.m_Indices.size() * sizeof(uint32_t), meshData.m_Indices.data(), GL_STATIC_DRAW);
        
        // 设置顶点属性指针
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MathLib::HVector3), (void*)0);
        glEnableVertexAttribArray(0);
        
        indicesCount = static_cast<int>(meshData.m_Indices.size());
        
        // 解绑
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        
        // 创建着色器
        shaderProgram = createShaderProgram(lineShaderSource, lineFragmentShaderSource);
    }
    
    void CleanUp() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
        glDeleteProgram(shaderProgram);
    }
};

GizmoRenderUnit::GizmoRenderUnit(const MathLib::GraphicUtils::MeshData32& meshData)
    : m_impl(new Impl()) {
    m_impl->CreateBuffers(meshData);
}

GizmoRenderUnit::~GizmoRenderUnit() {
    m_impl->CleanUp();
}

void GizmoRenderUnit::SetTransformation(const MathLib::HMatrix4* transform) {
    if (transform) {
        m_impl->transform = *transform;
    }
}

void GizmoRenderUnit::SetTransformation(const MathLib::HVector3* scale, const MathLib::HVector3* position) {
    if (scale) {
        m_impl->scale = *scale;
    }
    if (position) {
        m_impl->position = *position;
    }
    
    // 构建变换矩阵
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(m_impl->position[0], m_impl->position[1], m_impl->position[2]));
    model = glm::scale(model, glm::vec3(m_impl->scale[0], m_impl->scale[1], m_impl->scale[2]));
    
    m_impl->transform = *reinterpret_cast<MathLib::HMatrix4*>(&model);
}

void GizmoRenderUnit::UpdateTransformation() {
    // 如果需要动态更新变换矩阵，在这里实现
}

void GizmoRenderUnit::Show(bool show) {
    m_impl->visible = show;
}

void GizmoRenderUnit::Render(MathLib::GraphicUtils::Camera& camera) {
    if (!m_impl->visible) return;
    
    glUseProgram(m_impl->shaderProgram);
    
    // 设置着色器参数
    glUniformMatrix4fv(glGetUniformLocation(m_impl->shaderProgram, "model"), 1, GL_FALSE, m_impl->transform.data());
    
    // 假设Camera类提供了视图和投影矩阵
    MathLib::HMatrix4 viewMatrix = camera.GetViewMatrix();
    MathLib::HMatrix4 projMatrix = camera.GetProjectMatrix();
    
    glUniformMatrix4fv(glGetUniformLocation(m_impl->shaderProgram, "view"), 1, GL_FALSE, viewMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(m_impl->shaderProgram, "projection"), 1, GL_FALSE, projMatrix.data());
    
    // 设置颜色
    glUniform4fv(glGetUniformLocation(m_impl->shaderProgram, "lineColor"), 1, m_impl->color);
    
    // 线框模式
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    // 绘制
    glBindVertexArray(m_impl->VAO);
    glDrawElements(GL_LINES, m_impl->indicesCount, GL_UNSIGNED_INT, 0);
    
    // 恢复默认设置
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindVertexArray(0);
}

void GizmoRenderUnit::AddToScene(void* scene) {
    m_impl->sceneParent = scene;
    // 可能需要更多场景整合代码
}

void GizmoRenderUnit::RemoveFromScene() {
    m_impl->sceneParent = nullptr;
}

void GizmoRenderUnit::SetColor(const float* color) {
    if (color) {
        for (int i = 0; i < 4; i++) {
            m_impl->color[i] = color[i];
        }
    }
}