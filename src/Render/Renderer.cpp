#include "Renderer/Renderer.h"
#include "Renderer/RenderUnit.h"
#include <chrono>
#include <Math/GraphicUtils/Camara.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

// 实现Renderer类
class Renderer : public IRenderer {
public:
    Renderer(int argc, char** argv);
    ~Renderer() override;

    void Release() override;
    void SetApplicationName(const std::string& name) override;
    void SetUp(
        std::function<void(void*)> mousePressCb,
        std::function<void(void*)> mouseReleaseCb,
        std::function<void(void*)> mouseMoveCb,
        std::function<void(void*)> mouseScrollCb,
        std::function<void(void*)> keyPressCb,
        std::function<void(void*)> keyReleaseCb
    ) override;
    void AddRenderObject(std::shared_ptr<RenderObject> renderable) override;
    void RemoveRenderObject(std::shared_ptr<RenderObject> renderable) override;
    bool Tick() override;
    MathLib::GraphicUtils::Camera* GetActiveCamera() override;

private:
    void InitializeGL();
    void InitializeCamera();
    void Render();
    
    static void ErrorCallback(int error, const char* description);
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void CursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

private:
    GLFWwindow* m_window = nullptr;
    std::string m_appName = "Physics Renderer";
    int m_width = 1280;
    int m_height = 720;
    
    std::unique_ptr<MathLib::GraphicUtils::Camera> m_camera;
    std::vector<std::shared_ptr<RenderObject>> m_renderObjects;
    
    // 回调函数
    std::function<void(void*)> m_mousePressCb;
    std::function<void(void*)> m_mouseReleaseCb;
    std::function<void(void*)> m_mouseMoveCb;
    std::function<void(void*)> m_mouseScrollCb;
    std::function<void(void*)> m_keyPressCb;
    std::function<void(void*)> m_keyReleaseCb;
    
    // 鼠标状态
    bool m_mousePressed = false;
    double m_lastMouseX = 0;
    double m_lastMouseY = 0;
};

// Renderer实现
Renderer::Renderer(int argc, char** argv) {
    InitializeGL();
    InitializeCamera();
}

Renderer::~Renderer() {
    if (m_window) {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

void Renderer::InitializeGL() {
    // 初始化GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    glfwSetErrorCallback(ErrorCallback);
    
    // 设置OpenGL版本
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // 创建窗口
    m_window = glfwCreateWindow(m_width, m_height, m_appName.c_str(), nullptr, nullptr);
    if (!m_window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    
    glfwMakeContextCurrent(m_window);
    
    // 初始化GLEW
    GLenum err = glewInit();
    if (GLEW_OK != err) {
        std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl;
        exit(EXIT_FAILURE);
    }
    
    // 设置回调函数
    glfwSetWindowUserPointer(m_window, this);
    glfwSetKeyCallback(m_window, KeyCallback);
    glfwSetMouseButtonCallback(m_window, MouseButtonCallback);
    glfwSetCursorPosCallback(m_window, CursorPosCallback);
    glfwSetScrollCallback(m_window, ScrollCallback);
    glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow* window, int width, int height) {
        // 窗口大小改变回调
        glViewport(0, 0, width, height);
    });
    
    // 设置OpenGL状态
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
}

void Renderer::InitializeCamera() {
    m_camera = std::make_unique<MathLib::GraphicUtils::Camera>(
        MathLib::HVector3(50.0f, 50.0f, 50.0f),
        MathLib::HVector3(-0.6f, -0.2f, -0.7f),
        static_cast<MathLib::HReal>(m_width) / m_height
    );
    
    // 初始化鼠标位置为窗口中心
    m_lastMouseX = m_width / 2.0;
    m_lastMouseY = m_height / 2.0;
}

void Renderer::SetApplicationName(const std::string& name) {
    m_appName = name;
    if (m_window) {
        glfwSetWindowTitle(m_window, name.c_str());
    }
}

void Renderer::SetUp(
    std::function<void(void*)> mousePressCb,
    std::function<void(void*)> mouseReleaseCb,
    std::function<void(void*)> mouseMoveCb,
    std::function<void(void*)> mouseScrollCb,
    std::function<void(void*)> keyPressCb,
    std::function<void(void*)> keyReleaseCb
) {
    m_mousePressCb = mousePressCb;
    m_mouseReleaseCb = mouseReleaseCb;
    m_mouseMoveCb = mouseMoveCb;
    m_mouseScrollCb = mouseScrollCb;
    m_keyPressCb = keyPressCb;
    m_keyReleaseCb = keyReleaseCb;
}

void Renderer::AddRenderObject(std::shared_ptr<RenderObject> renderable) {
    m_renderObjects.push_back(renderable);
}

void Renderer::RemoveRenderObject(std::shared_ptr<RenderObject> renderable) {
    auto it = std::find(m_renderObjects.begin(), m_renderObjects.end(), renderable);
    if (it != m_renderObjects.end()) {
        m_renderObjects.erase(it);
    }
}

bool Renderer::Tick() {
    if (glfwWindowShouldClose(m_window)) {
        return false;
    }
    
    // 更新渲染对象
    for (auto& obj : m_renderObjects) {
        obj->UpdateTransform();
    }
    
    // 渲染
    Render();
    
    // 交换缓冲区并处理事件
    glfwSwapBuffers(m_window);
    glfwPollEvents();
    
    return true;
}

MathLib::GraphicUtils::Camera* Renderer::GetActiveCamera() {
    return m_camera.get();
}

void Renderer::Render() {
    // 清除缓冲区
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 设置相机
    int width, height;
    glfwGetFramebufferSize(m_window, &width, &height);
    glViewport(0, 0, width, height);
    
    // 相机宽高比可能需要更新
    float aspectRatio = static_cast<float>(width) / height;
    if (m_camera && fabs(m_camera->GetAspectRatio() - aspectRatio) > 0.01f) {
        // 如果宽高比有明显变化，我们可能需要更新相机的投影矩阵
        // 但是这里依赖于Camera类的实现方式
    }
    
    // 渲染所有对象
    for (auto& obj : m_renderObjects) {
        if (obj) {
            obj->Render(*m_camera);
        }
    }
}

// 静态回调函数实现
void Renderer::ErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

void Renderer::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer) {
        // 处理相机控制键
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            if (renderer->m_camera) {
                // 根据按键移动相机
                float speed = 0.5f;
                char cameraKey = 0;
                
                // 将GLFW键码转换为Camera类期望的键值
                switch (key) {
                    case GLFW_KEY_W: cameraKey = 'W'; break;
                    case GLFW_KEY_S: cameraKey = 'S'; break;
                    case GLFW_KEY_A: cameraKey = 'A'; break;
                    case GLFW_KEY_D: cameraKey = 'D'; break;
                    case GLFW_KEY_Q: cameraKey = 'Q'; break;
                    case GLFW_KEY_E: cameraKey = 'E'; break;
                    default: cameraKey = 0; break;
                }
                
                if (cameraKey) {
                    renderer->m_camera->HandleKey(cameraKey, 0, 0, speed);
                }
            }
        }
        
        if (action == GLFW_PRESS && renderer->m_keyPressCb) {
            renderer->m_keyPressCb(reinterpret_cast<void*>(&key));
        } else if (action == GLFW_RELEASE && renderer->m_keyReleaseCb) {
            renderer->m_keyReleaseCb(reinterpret_cast<void*>(&key));
        }
    }
}

void Renderer::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer) {
        if (action == GLFW_PRESS) {
            renderer->m_mousePressed = true;
            // 获取当前鼠标位置
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            renderer->m_lastMouseX = xpos;
            renderer->m_lastMouseY = ypos;
            
            if (renderer->m_mousePressCb) {
                renderer->m_mousePressCb(reinterpret_cast<void*>(&button));
            }
        } else if (action == GLFW_RELEASE) {
            renderer->m_mousePressed = false;
            if (renderer->m_mouseReleaseCb) {
                renderer->m_mouseReleaseCb(reinterpret_cast<void*>(&button));
            }
        }
    }
}

void Renderer::CursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer) {
        // 计算鼠标移动
        double deltaX = xpos - renderer->m_lastMouseX;
        double deltaY = ypos - renderer->m_lastMouseY;
        
        // 更新相机
        if (renderer->m_camera && renderer->m_mousePressed) {
            renderer->m_camera->HandleMotion(xpos, ypos);
        }
        
        // 调用回调函数
        if (renderer->m_mouseMoveCb && renderer->m_mousePressed) {
            // 创建临时数据结构
            struct MouseMoveData {
                double xpos, ypos, deltaX, deltaY;
            } data = {xpos, ypos, deltaX, deltaY};
            
            renderer->m_mouseMoveCb(reinterpret_cast<void*>(&data));
        }
    }
    
    // 保存鼠标位置
    if (renderer) {
        renderer->m_lastMouseX = xpos;
        renderer->m_lastMouseY = ypos;
    }
}

void Renderer::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    if (renderer) {
        // 更新相机
        if (renderer->m_camera) {
            // 相机缩放 - 调整相机速度而不是直接变换
            float speed = renderer->m_camera->GetSpeed();
            speed += yoffset * 0.1f;
            if (speed < 0.1f) speed = 0.1f;
            if (speed > 10.0f) speed = 10.0f;
            renderer->m_camera->SetSpeed(speed);
        }
        
        // 调用回调函数
        if (renderer->m_mouseScrollCb) {
            struct ScrollData {
                double xoffset, yoffset;
            } data = {xoffset, yoffset};
            
            renderer->m_mouseScrollCb(reinterpret_cast<void*>(&data));
        }
    }
}

void Renderer::Release() {
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    
    // 清空渲染对象列表
    m_renderObjects.clear();
    
    // 释放相机资源
    m_camera.reset();
    
    // 终止GLFW
    glfwTerminate();
}

// 创建渲染器实例
IRenderer* CreateRenderer(int argc, char** argv) {
    return new Renderer(argc, argv);
} 