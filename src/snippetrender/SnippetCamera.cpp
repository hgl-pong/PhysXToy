#include "SnippetCamera.h"
#include <ctype.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

namespace MathLib
{
    Camera::Camera(const HVector3& eye, const HVector3& dir) : mMouseX(0), mMouseY(0), mSpeed(2.0f)
    {
        mEye = eye;
        mDir = dir.normalized();
    }

    void Camera::handleMouse(int button, int state, int x, int y)
    {
        (void)state;
        (void)button;
        mMouseX = x;
        mMouseY = y;
    }

    bool Camera::handleKey(unsigned char key, int x, int y, MathLib::HReal speed)
    {
        (void)x;
        (void)y;

        const HVector3 viewY = mDir.cross(HVector3(0, 1, 0)).normalized();
        switch (toupper(key))
        {
        case 'W':   mEye += mDir * mSpeed * speed; break;
        case 'S':   mEye -= mDir * mSpeed * speed; break;
        case 'A':   mEye -= viewY * mSpeed * speed; break;
        case 'D':   mEye += viewY * mSpeed * speed; break;
        default:                                return false;
        }
        return true;
    }

    void Camera::handleAnalogMove(MathLib::HReal x, MathLib::HReal y)
    {
        HVector3 viewY = mDir.cross(HVector3(0, 1, 0)).normalized();
        mEye += mDir * y;
        mEye += viewY * x;
    }

    void Camera::handleMotion(int x, int y)
    {
        const int dx = mMouseX - x;
        const int dy = mMouseY - y;

        const HVector3 viewY = mDir.cross(HVector3(0, 1, 0)).normalized();

        const MathLib::HReal Sensitivity = H_PI * 0.5f / 180.0f;

        const AngleAxisf qx(Sensitivity * dx, HVector3(0, 1, 0));
        mDir = qx * mDir;
        const AngleAxisf qy(Sensitivity * dy, viewY);
        mDir = qy * mDir;

        mDir.normalize();

        mMouseX = x;
        mMouseY = y;
    }

    HTransform3 Camera::getTransform()
    {
        HVector3 viewY = mDir.cross(HVector3(0, 1, 0));

        if (viewY.norm() < 1e-6f)
            return HTransform3(Translation3f(mEye));

        const HMatrix3 m = (HMatrix3() << mDir.cross(viewY), viewY, -mDir).finished();
        return HTransform3(Translation3f(mEye) * HQuaternion(m));
    }

    HVector3 Camera::getEye() const
    {
        return mEye;
    }

    HVector3 Camera::getDir() const
    {
        return mDir;
    }

    void Camera::setPose(const HVector3& eye, const HVector3& dir)
    {
        mEye = eye;
        mDir = dir.normalized();
    }

    void Camera::setSpeed(MathLib::HReal speed)
    {
        mSpeed = speed;
    }
}