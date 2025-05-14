#include "Renderer/Renderer.h"
#include "Renderer/RenderUnit.h"
#include <chrono>
#include "Math/GraphicUtils/Camara.h"
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <iostream>
#include <d3d11.h>
#include <DirectXMath.h>
#include <d3dcompiler.h>
#include <vector>
#include <wrl/client.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_dx11.h>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

using Microsoft::WRL::ComPtr;

ComPtr<ID3D11Device> g_d3dDevice;
ComPtr<ID3D11DeviceContext> g_d3dDeviceContext;

IRenderer* g_Renderer = nullptr;

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
	void AddGUIPanel(std::shared_ptr<GUIPanel> panel) override;
	void RemoveGUIPanel(std::shared_ptr<GUIPanel> panel) override;
	bool Tick() override;
	MathLib::GraphicUtils::Camera* GetActiveCamera() override;

private:
	void InitializeGLFW();
	void InitializeDirectX();
	void InitializeCamera();
	void InitializeImGui();
	void RenderImGui();
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

	std::function<void(void*)> m_mousePressCb;
	std::function<void(void*)> m_mouseReleaseCb;
	std::function<void(void*)> m_mouseMoveCb;
	std::function<void(void*)> m_mouseScrollCb;
	std::function<void(void*)> m_keyPressCb;
	std::function<void(void*)> m_keyReleaseCb;

	bool m_mousePressed = false;
	double m_lastMouseX = 0;
	double m_lastMouseY = 0;

	ComPtr<ID3D11Device> m_device;
	ComPtr<ID3D11DeviceContext> m_deviceContext;
	ComPtr<IDXGISwapChain> m_swapChain;
	ComPtr<ID3D11RenderTargetView> m_renderTargetView;
	ComPtr<ID3D11Texture2D> m_depthStencilBuffer;
	ComPtr<ID3D11DepthStencilView> m_depthStencilView;
	ComPtr<ID3D11DepthStencilState> m_depthStencilState;
	ComPtr<ID3D11RasterizerState> m_rasterizerState;
	DirectX::XMFLOAT4 m_clearColor = { 0.1f, 0.1f, 0.1f, 1.0f };

	bool m_imguiInitialized = false;
	bool m_showDemoWindow = false;
	bool m_showStatsWindow = true;
	
	// Track if dockspace layout has been initialized
	bool m_dockspaceInitialized = false;

	std::vector<std::shared_ptr<GUIPanel>> m_guiPanels;
};

IRenderer* GetRenderer()
{
	return g_Renderer;
}

Renderer::Renderer(int argc, char** argv) {
	InitializeGLFW();
	InitializeDirectX();
	InitializeCamera();
	InitializeImGui();
	_ASSERT(g_Renderer == nullptr);
	g_Renderer = this;
}

Renderer::~Renderer() {
	// 清理ImGui
	if (m_imguiInitialized) {
		ImGui_ImplDX11_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
	}

	if (m_window) {
		glfwDestroyWindow(m_window);
	}
	glfwTerminate();
	g_Renderer = nullptr;
}

void Renderer::InitializeGLFW() {
	if (!glfwInit()) {
		std::cerr << "Failed to initialize GLFW" << std::endl;
		exit(EXIT_FAILURE);
	}

	glfwSetErrorCallback(ErrorCallback);

	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

	m_window = glfwCreateWindow(m_width, m_height, m_appName.c_str(), nullptr, nullptr);
	if (!m_window) {
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwSetWindowUserPointer(m_window, this);

	glfwSetKeyCallback(m_window, KeyCallback);
	glfwSetMouseButtonCallback(m_window, MouseButtonCallback);
	glfwSetCursorPosCallback(m_window, CursorPosCallback);
	glfwSetScrollCallback(m_window, ScrollCallback);

}

void Renderer::InitializeDirectX() {
	HWND hwnd = glfwGetWin32Window(m_window);

	DXGI_SWAP_CHAIN_DESC swapChainDesc = {};
	swapChainDesc.BufferCount = 1;
	swapChainDesc.BufferDesc.Width = m_width;
	swapChainDesc.BufferDesc.Height = m_height;
	swapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	swapChainDesc.BufferDesc.RefreshRate.Numerator = 60;
	swapChainDesc.BufferDesc.RefreshRate.Denominator = 1;
	swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	swapChainDesc.OutputWindow = hwnd;
	swapChainDesc.SampleDesc.Count = 1;
	swapChainDesc.SampleDesc.Quality = 0;
	swapChainDesc.Windowed = TRUE;

	D3D_FEATURE_LEVEL featureLevel;
	UINT createDeviceFlags = 0;

#ifdef _DEBUG
	createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

	HRESULT hr = D3D11CreateDeviceAndSwapChain(
		nullptr,
		D3D_DRIVER_TYPE_HARDWARE,
		nullptr,
		createDeviceFlags,
		nullptr,
		0,
		D3D11_SDK_VERSION,
		&swapChainDesc,
		&m_swapChain,
		&m_device,
		&featureLevel,
		&m_deviceContext
	);

	if (FAILED(hr)) {
		std::cerr << "Failed to create DirectX device and swap chain" << std::endl;
		exit(EXIT_FAILURE);
	}

	g_d3dDevice = m_device;
	g_d3dDeviceContext = m_deviceContext;

	ComPtr<ID3D11Texture2D> backBuffer;
	hr = m_swapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), &backBuffer);
	if (FAILED(hr)) {
		std::cerr << "Failed to get back buffer" << std::endl;
		exit(EXIT_FAILURE);
	}

	hr = m_device->CreateRenderTargetView(backBuffer.Get(), nullptr, &m_renderTargetView);
	if (FAILED(hr)) {
		std::cerr << "Failed to create render target view" << std::endl;
		exit(EXIT_FAILURE);
	}

	D3D11_TEXTURE2D_DESC depthStencilDesc = {};
	depthStencilDesc.Width = m_width;
	depthStencilDesc.Height = m_height;
	depthStencilDesc.MipLevels = 1;
	depthStencilDesc.ArraySize = 1;
	depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	depthStencilDesc.SampleDesc.Count = 1;
	depthStencilDesc.SampleDesc.Quality = 0;
	depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
	depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	depthStencilDesc.CPUAccessFlags = 0;
	depthStencilDesc.MiscFlags = 0;

	hr = m_device->CreateTexture2D(&depthStencilDesc, nullptr, &m_depthStencilBuffer);
	if (FAILED(hr)) {
		std::cerr << "Failed to create depth stencil buffer" << std::endl;
		exit(EXIT_FAILURE);
	}

	hr = m_device->CreateDepthStencilView(m_depthStencilBuffer.Get(), nullptr, &m_depthStencilView);
	if (FAILED(hr)) {
		std::cerr << "Failed to create depth stencil view" << std::endl;
		exit(EXIT_FAILURE);
	}

	D3D11_DEPTH_STENCIL_DESC dsDesc = {};
	dsDesc.DepthEnable = TRUE;
	dsDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
	dsDesc.DepthFunc = D3D11_COMPARISON_LESS;
	dsDesc.StencilEnable = FALSE;

	hr = m_device->CreateDepthStencilState(&dsDesc, &m_depthStencilState);
	if (FAILED(hr)) {
		std::cerr << "Failed to create depth stencil state" << std::endl;
		exit(EXIT_FAILURE);
	}

	m_deviceContext->OMSetDepthStencilState(m_depthStencilState.Get(), 1);

	D3D11_RASTERIZER_DESC rasterizerDesc = {};
	rasterizerDesc.FillMode = D3D11_FILL_SOLID;
	rasterizerDesc.CullMode = D3D11_CULL_NONE;
	rasterizerDesc.FrontCounterClockwise = FALSE;
	rasterizerDesc.DepthBias = 0;
	rasterizerDesc.DepthBiasClamp = 0.0f;
	rasterizerDesc.SlopeScaledDepthBias = 0.0f;
	rasterizerDesc.DepthClipEnable = TRUE;
	rasterizerDesc.ScissorEnable = FALSE;
	rasterizerDesc.MultisampleEnable = FALSE;
	rasterizerDesc.AntialiasedLineEnable = FALSE;

	hr = m_device->CreateRasterizerState(&rasterizerDesc, &m_rasterizerState);
	if (FAILED(hr)) {
		std::cerr << "Failed to create rasterizer state" << std::endl;
		exit(EXIT_FAILURE);
	}

	m_deviceContext->RSSetState(m_rasterizerState.Get());

	m_deviceContext->OMSetRenderTargets(1, m_renderTargetView.GetAddressOf(), m_depthStencilView.Get());

	D3D11_VIEWPORT viewport = {};
	viewport.TopLeftX = 0.0f;
	viewport.TopLeftY = 0.0f;
	viewport.Width = static_cast<float>(m_width);
	viewport.Height = static_cast<float>(m_height);
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;

	m_deviceContext->RSSetViewports(1, &viewport);
}

void Renderer::InitializeCamera() {
	m_camera = std::make_unique<MathLib::GraphicUtils::Camera>(
		MathLib::HVector3(50.0f, 50.0f, 50.0f),
		MathLib::HVector3(-0.6f, -0.2f, -0.7f),
		static_cast<MathLib::HReal>(m_width) / m_height
	);

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

void Renderer::AddGUIPanel(std::shared_ptr<GUIPanel> panel) {
	m_guiPanels.push_back(panel);
}

void Renderer::RemoveGUIPanel(std::shared_ptr<GUIPanel> panel) {
	auto it = std::find(m_guiPanels.begin(), m_guiPanels.end(), panel);
	if (it != m_guiPanels.end()) {
		m_guiPanels.erase(it);
	}
}

bool Renderer::Tick() {
	if (glfwWindowShouldClose(m_window)) {
		return false;
	}

	for (auto& obj : m_renderObjects) {
		obj->UpdateTransform();
	}

	Render();

	m_swapChain->Present(1, 0);
	glfwPollEvents();

	return true;
}

MathLib::GraphicUtils::Camera* Renderer::GetActiveCamera() {
	return m_camera.get();
}

void Renderer::Render() {
	int width, height;
	glfwGetFramebufferSize(m_window, &width, &height);
	if (width != m_width || height != m_height) {
		m_width = width;
		m_height = height;

		m_renderTargetView.Reset();
		m_depthStencilView.Reset();
		m_depthStencilBuffer.Reset();

		HRESULT hr = m_swapChain->ResizeBuffers(1, width, height, DXGI_FORMAT_R8G8B8A8_UNORM, 0);
		if (FAILED(hr)) {
			std::cerr << "Failed to resize swap chain" << std::endl;
			return;
		}

		ComPtr<ID3D11Texture2D> backBuffer;
		hr = m_swapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), &backBuffer);
		if (FAILED(hr)) {
			std::cerr << "Failed to get back buffer" << std::endl;
			return;
		}

		hr = m_device->CreateRenderTargetView(backBuffer.Get(), nullptr, &m_renderTargetView);
		if (FAILED(hr)) {
			std::cerr << "Failed to create render target view" << std::endl;
			return;
		}

		D3D11_TEXTURE2D_DESC depthStencilDesc = {};
		depthStencilDesc.Width = width;
		depthStencilDesc.Height = height;
		depthStencilDesc.MipLevels = 1;
		depthStencilDesc.ArraySize = 1;
		depthStencilDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
		depthStencilDesc.SampleDesc.Count = 1;
		depthStencilDesc.SampleDesc.Quality = 0;
		depthStencilDesc.Usage = D3D11_USAGE_DEFAULT;
		depthStencilDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
		depthStencilDesc.CPUAccessFlags = 0;
		depthStencilDesc.MiscFlags = 0;

		hr = m_device->CreateTexture2D(&depthStencilDesc, nullptr, &m_depthStencilBuffer);
		if (FAILED(hr)) {
			std::cerr << "Failed to create depth stencil buffer" << std::endl;
			return;
		}

		hr = m_device->CreateDepthStencilView(m_depthStencilBuffer.Get(), nullptr, &m_depthStencilView);
		if (FAILED(hr)) {
			std::cerr << "Failed to create depth stencil view" << std::endl;
			return;
		}

		m_deviceContext->OMSetRenderTargets(1, m_renderTargetView.GetAddressOf(), m_depthStencilView.Get());

		D3D11_VIEWPORT viewport = {};
		viewport.TopLeftX = 0.0f;
		viewport.TopLeftY = 0.0f;
		viewport.Width = static_cast<float>(width);
		viewport.Height = static_cast<float>(height);
		viewport.MinDepth = 0.0f;
		viewport.MaxDepth = 1.0f;

		m_deviceContext->RSSetViewports(1, &viewport);

		if (m_camera) {
			float aspectRatio = static_cast<float>(width) / height;
			if (fabs(m_camera->GetAspectRatio() - aspectRatio) > 0.01f) {
				MathLib::HVector3 eye = m_camera->GetEye();
				MathLib::HVector3 dir = m_camera->GetDir();
				m_camera = std::make_unique<MathLib::GraphicUtils::Camera>(
					eye, dir, aspectRatio
				);
			}
		}
	}

	float clearColor[4] = { m_clearColor.x, m_clearColor.y, m_clearColor.z, m_clearColor.w };
	m_deviceContext->ClearRenderTargetView(m_renderTargetView.Get(), clearColor);
	m_deviceContext->ClearDepthStencilView(m_depthStencilView.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	m_deviceContext->OMSetRenderTargets(1, m_renderTargetView.GetAddressOf(), m_depthStencilView.Get());

	for (auto& obj : m_renderObjects) {
		if (obj) {
			obj->Render(*m_camera);
		}
	}

	RenderImGui();
}

void Renderer::Release() {
	if (m_imguiInitialized) {
		ImGui_ImplDX11_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
		m_imguiInitialized = false;
	}

	g_d3dDeviceContext.Reset();
	g_d3dDevice.Reset();

	m_renderTargetView.Reset();
	m_depthStencilView.Reset();
	m_depthStencilBuffer.Reset();
	m_depthStencilState.Reset();
	m_rasterizerState.Reset();
	m_deviceContext.Reset();
	m_swapChain.Reset();
	m_device.Reset();

	if (m_window) {
		glfwDestroyWindow(m_window);
		m_window = nullptr;
	}
	glfwTerminate();
}

void Renderer::ErrorCallback(int error, const char* description) {
	std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

void Renderer::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
	if (renderer) {
		ImGuiIO& io = ImGui::GetIO();
		if (io.WantCaptureKeyboard) {
			return;
		}

		if (action == GLFW_PRESS || action == GLFW_REPEAT) {
			if (renderer->m_camera) {
				float speed = 0.5f;
				char cameraKey = 0;

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

		if (renderer->m_keyPressCb && action == GLFW_PRESS) {
			struct KeyData {
				int key, scancode, mods;
			} data = { key, scancode, mods };
			renderer->m_keyPressCb(&data);
		}
		else if (renderer->m_keyReleaseCb && action == GLFW_RELEASE) {
			struct KeyData {
				int key, scancode, mods;
			} data = { key, scancode, mods };
			renderer->m_keyReleaseCb(&data);
		}
	}
}

void Renderer::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
	Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
	if (renderer) {
		ImGuiIO& io = ImGui::GetIO();
		if (io.WantCaptureMouse) {
			return;
		}

		if (action == GLFW_PRESS) {
			renderer->m_mousePressed = true;
		}
		else if (action == GLFW_RELEASE) {
			renderer->m_mousePressed = false;
		}

		if (action == GLFW_PRESS && renderer->m_mousePressCb) {
			struct MouseButtonData {
				int button, mods;
			} data = { button, mods };
			renderer->m_mousePressCb(&data);
		}
		else if (action == GLFW_RELEASE && renderer->m_mouseReleaseCb) {
			struct MouseButtonData {
				int button, mods;
			} data = { button, mods };
			renderer->m_mouseReleaseCb(&data);
		}
	}
}

void Renderer::CursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
	Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
	if (renderer) {
		ImGuiIO& io = ImGui::GetIO();
		if (io.WantCaptureMouse) {
			return;
		}

		if (renderer->m_mousePressed && renderer->m_camera) {
			double deltaX = xpos - renderer->m_lastMouseX;
			double deltaY = ypos - renderer->m_lastMouseY;
			renderer->m_lastMouseX = xpos;
			renderer->m_lastMouseY = ypos;
			if (renderer->m_mousePressed && renderer->m_camera) {
				renderer->m_camera->HandleMotion(xpos, ypos);
			}

			if (renderer->m_mouseMoveCb) {
				struct MouseMoveData {
					double xpos, ypos, deltaX, deltaY;
				} data = { xpos, ypos, deltaX, deltaY };
				renderer->m_mouseMoveCb(&data);
			}
		}
	}
}

void Renderer::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
	Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
	if (renderer) {
		ImGuiIO& io = ImGui::GetIO();
		if (io.WantCaptureMouse) {
			return;
		}

		if (renderer->m_camera) {
			float speed = renderer->m_camera->GetSpeed();
			speed += yoffset * 0.1f;
			if (speed < 0.1f) speed = 0.1f;
			if (speed > 10.0f) speed = 10.0f;
			renderer->m_camera->SetSpeed(speed);
		}

		if (renderer->m_mouseScrollCb) {
			struct ScrollData {
				double xoffset, yoffset;
			} data = { xoffset, yoffset };
			renderer->m_mouseScrollCb(&data);
		}
	}
}

void Renderer::InitializeImGui() {
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	(void)io;

	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;    // Enable Docking
	io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;  // Enable Multi-Viewport / Platform Windows

	// Setup ImGui style
	ImGui::StyleColorsDark();

	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones
	ImGuiStyle& style = ImGui::GetStyle();
	if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
		style.WindowRounding = 0.0f;
		style.Colors[ImGuiCol_WindowBg].w = 1.0f;
	}

	ImGui_ImplGlfw_InitForOther(m_window, true);
	ImGui_ImplDX11_Init(m_device.Get(), m_deviceContext.Get());

	m_imguiInitialized = true;
}

void Renderer::RenderImGui() {
	if (!m_imguiInitialized)
		return;

	ImGui_ImplDX11_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// Create a fullscreen window for the dockspace
	static bool dockspaceOpen = true;
	static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;

	// We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
	// because it would be confusing to have two docking targets within each other.
	ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
	
	// Make the parent window background fully transparent
	ImGuiViewport* viewport = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(viewport->Pos);
	ImGui::SetNextWindowSize(viewport->Size);
	ImGui::SetNextWindowViewport(viewport->ID);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
	window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
	
	// When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
	// and handle the pass-thru hole, so we ask Begin() to not render a background.
	if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
		window_flags |= ImGuiWindowFlags_NoBackground;

	// Important: note that we proceed even if Begin() returns false (aka window is collapsed).
	// This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
	// all active windows docked into it will lose their parent and become undocked.
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::Begin(m_appName.c_str(), &dockspaceOpen, window_flags);
	ImGui::PopStyleVar();
	ImGui::PopStyleVar(2);

	// DockSpace
	ImGuiIO& io = ImGui::GetIO();
	ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");
	if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable) {
		ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
	}

	// Menu Bar
	if (ImGui::BeginMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("Exit", "Alt+F4")) {
				glfwSetWindowShouldClose(m_window, GLFW_TRUE);
			}
			ImGui::EndMenu();
		}
        
		if (ImGui::BeginMenu("View")) {
			ImGui::MenuItem("Stats Window", nullptr, &m_showStatsWindow);
			ImGui::MenuItem("Demo Window", nullptr, &m_showDemoWindow);
			ImGui::EndMenu();
		}
		
		ImGui::EndMenuBar();
	}

	ImGui::End(); // End of DockSpace window

	if (m_showDemoWindow)
		ImGui::ShowDemoWindow(&m_showDemoWindow);

	if (m_showStatsWindow) {
		ImGui::Begin("Renderer Stats", &m_showStatsWindow);

		ImGui::Text("Application Name: %s", m_appName.c_str());
		ImGui::Text("Window Size: %d x %d", m_width, m_height);
		ImGui::Text("Render Object Count: %zu", m_renderObjects.size());

		if (m_camera) {
			MathLib::HVector3 eye = m_camera->GetEye();
			MathLib::HVector3 dir = m_camera->GetDir();

			ImGui::Separator();
			ImGui::Text("Camera Info:");
			ImGui::Text("Position: (%.2f, %.2f, %.2f)", eye[0], eye[1], eye[2]);
			ImGui::Text("Direction: (%.2f, %.2f, %.2f)", dir[0], dir[1], dir[2]);

			if (ImGui::Button("Reset Camera")) {
				m_camera = std::make_unique<MathLib::GraphicUtils::Camera>(
					MathLib::HVector3(50.0f, 50.0f, 50.0f),
					MathLib::HVector3(-0.6f, -0.2f, -0.7f),
					static_cast<MathLib::HReal>(m_width) / m_height
				);
			}
		}

		ImGui::Separator();
		ImGui::ColorEdit3("Background Color", &m_clearColor.x);
		ImGui::Checkbox("Show ImGui Demo Window", &m_showDemoWindow);

		ImGui::End();
	}

	for (auto& panel : m_guiPanels) {
		if (panel) {
			panel->Render();
		}
	}

	ImGui::Render();
	ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
	
	// Update and Render additional Platform Windows
	// (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
	//  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
	if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
		// Restore the DX11 render target
		//m_deviceContext->OMSetRenderTargets(1, m_renderTargetView.GetAddressOf(), m_depthStencilView.Get());
	}
}

IRenderer* CreateRenderer(int argc, char** argv) {
	try {
		return new Renderer(argc, argv);
	}
	catch (const std::exception& e) {
		std::cerr << "Failed to create renderer: " << e.what() << std::endl;
		return nullptr;
	}
}