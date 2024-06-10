#pragma once
#include <functional>
#include <Magnum/Magnum.h>
#include <Magnum/Platform/Sdl2Application.h>
namespace MagnumRender
{
    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseEvent&)> MousePresseCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseEvent&)> MouseReleaseCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseMoveEvent&)> MouseMotionCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::MouseScrollEvent&)> MouseScrollCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::KeyEvent&)> KeyBoardPressCallback;
    typedef std::function<void(Magnum::Platform::Sdl2Application::KeyEvent&)> KeyBoardReleaseCallback;

    class Renderer
    {
    public:
        virtual void SetUp(MousePresseCallback& mousePressCallback,
            MouseReleaseCallback& mouseReleaseCallback,
            MouseMotionCallback& mouseMotioCallback,
            MouseScrollCallback& mouseScrollCallback,
            KeyBoardPressCallback& keyboardPressCallback,
            KeyBoardReleaseCallback& keyBoardCallback) = 0;
        virtual void Release() = 0;
        virtual int Run() = 0;
        virtual void SetApplicationName(const char* name) = 0;
    };

    extern Renderer*CreateRenderer(int argc, char **argv);
}