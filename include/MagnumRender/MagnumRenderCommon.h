#pragma once
#include <functional>
#include <Magnum/Magnum.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/SceneGraph/Object.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Math/GraphicUtils/Camara.h>
namespace MagnumRender
{
    using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;
    using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;

    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseEvent &)> MousePresseCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseEvent &)> MouseReleaseCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseMoveEvent &)> MouseMotionCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseScrollEvent &)> MouseScrollCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::KeyEvent &)> KeyBoardPressCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::KeyEvent &)> KeyBoardReleaseCallback;

    class RenderObject;
    class Renderer
    {
    public:
        virtual void SetUp(const MousePresseCallback &mousePressCallback,
                           const MouseReleaseCallback &mouseReleaseCallback,
                           const MouseMotionCallback &mouseMotioCallback,
                           const MouseScrollCallback &mouseScrollCallback,
                           const KeyBoardPressCallback &keyboardPressCallback,
                           const KeyBoardReleaseCallback &keyBoardCallback) = 0;
        virtual void Release() = 0;
        virtual int Tick() = 0;
        virtual void SetApplicationName(const char *name) = 0;
        virtual void AddRenderObject(std::shared_ptr<RenderObject> &renderObject) = 0;
        virtual void RemoveRenderObject(std::shared_ptr<RenderObject> &renderobject) = 0;
        virtual MathLib::GraphicUtils::Camera *GetActiveCamera() = 0;
    };

    class RenderObject
    {
    public:
        virtual void UpdateTransform() = 0;
        virtual void ShowWireframe(bool show) = 0;
        virtual void ShowBoundingBox(bool show) = 0;
        virtual void Show(bool show) = 0;
        virtual void UseWorldBoundingBox(bool useWorldBoundingBox) = 0;
        virtual MathLib::HAABBox3D GetWorldBoundingBox() const = 0;
        virtual void Render(MathLib::GraphicUtils::Camera &camera) = 0;
        virtual void AddToScene(Scene3D &scene) = 0;
        virtual void RemoveFromScene() = 0;
    };

    extern Renderer *CreateRenderer(int argc, char **argv);
}