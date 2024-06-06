#include "Camera.h"
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

    HMatrix4 Perspective(float fov, float aspect, float near, float far) {
        float S = 1.0f / std::tanf(fov/2 * MathLib::H_PI / 180.f);

        HMatrix4 m = HMatrix4::Zero();
        m(0, 0) = S / aspect;
        m(1, 1) = S;
        m(2, 2) = -(far + near) / (far - near);
        m(2, 3) = -1.0f;
        m(3, 2) = -(2.0f * far * near) / (far - near);
        m(3, 3) = 0.0f;

        return m;
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

        const MathLib::HAngleAxis qx(Sensitivity * dx, HVector3(0, 1, 0));
        mDir = qx * mDir;
        const MathLib::HAngleAxis qy(Sensitivity * dy, viewY);
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
            return HTransform3(MathLib::HTranslation3(mEye));

        const HMatrix3 m = (HMatrix3() << mDir.cross(viewY), viewY, -mDir).finished();
        return HTransform3(MathLib::HTranslation3(mEye) * HQuaternion(m));
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
        mViewProjectMatrix = (mProjectMatrix.transpose() * mViewMatrix.transpose()).transpose();
    }
}