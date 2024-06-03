#include "Camera.h"
using namespace Eigen;

namespace MathLib
{
    HMatrix4 LookAt(const HVector3& eye, const HVector3& target, const HVector3& up) {
        const HVector3 backward = (eye - target).normalized();
        const HVector3 right = up.cross(backward).normalized();
        const HVector3 realUp = backward.cross(right);
        HMatrix4 matrix;
        matrix << right[0], right[1], right[2],0,
			realUp[0], realUp[1], realUp[2],0,
			backward[0], backward[1], backward[2],0,
			eye[0], eye[1], eye[2],1;
        return matrix;
    }

    HMatrix4 Perspective(MathLib::HReal fovy, MathLib::HReal& aspectRatio, MathLib::HReal zNear, MathLib::HReal zFar) {
        HVector2 scaleXY = - 2 * zNear * tan(fovy / 2) * 0.5 * HVector2(1, 1 / aspectRatio);
		HMatrix4 matrix;
		matrix << scaleXY[0] , 0, 0, 0,
			0, scaleXY[1], 0, 0,
			0, 0, (zNear + zFar) / (zNear - zFar), -1,
            0, 0, 2 * zNear * zFar / (zNear - zFar), 0;
		return matrix;
	}

    Camera::Camera(const HVector3& eye, const HVector3& dir,  const MathLib::HReal& aspectRatio) : mMouseX(0), mMouseY(0), mSpeed(2.0f), mAspectRatio(aspectRatio)
    {
        mEye = eye;
        mDir = dir.normalized();
        _UpdateMatrix();
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
        case 'Q':   mEye += HVector3(0, 1, 0) * mSpeed * speed; break;
        case 'E':   mEye -= HVector3(0, 1, 0) * mSpeed * speed; break;
        default:                                return false;
        }
        _UpdateMatrix();
        return true;
    }

    void Camera::handleAnalogMove(MathLib::HReal x, MathLib::HReal y)
    {
        HVector3 viewY = mDir.cross(HVector3(0, 1, 0)).normalized();
        mEye += mDir * y;
        mEye += viewY * x;
        _UpdateMatrix();
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

        _UpdateMatrix();
    }

    HTransform3 Camera::getTransform()
    {
        HVector3 viewY = mDir.cross(HVector3(0, 1, 0));

        if (viewY.norm() < 1e-6f)
            return HTransform3(Translation3f(mEye));

        const HMatrix3 m = (HMatrix3() << mDir.cross(viewY), viewY, -mDir).finished();
        return HTransform3(Translation3f(mEye) * HQuaternion(m));
    }

    HMatrix4 Camera::getViewMatrix()
    {
        return mViewMatrix;
    }

    HMatrix4 Camera::getProjectMatrix()
    {
        return mProjectMatrix;
    }

    HMatrix4 Camera::getViewProjectMatrix()
    {
        return mViewProjectMatrix;
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

    void Camera::_UpdateMatrix()
    {
        mViewMatrix = LookAt(mEye, mEye + mDir, HVector3(0, 1, 0));
        mProjectMatrix = Perspective(mFOV, mAspectRatio, mNearClip, mFarClip);
        mViewProjectMatrix = mViewMatrix * mProjectMatrix;
    }
}