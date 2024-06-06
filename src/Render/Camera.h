#pragma once
#include "Physics/PhysicsTypes.h"
namespace MathLib
{
	class Camera
	{
	public:
		Camera(const HVector3 &eye, const HVector3 &dir, const MathLib::HReal &aspectRatio);

		void handleMouse(int button, int state, int x, int y);
		bool handleKey(unsigned char key, int x, int y, MathLib::HReal speed = 0.5f);
		void handleMotion(int x, int y);
		void handleAnalogMove(MathLib::HReal x, MathLib::HReal y);

		HVector3 getEye() const;
		HVector3 getDir() const;
		HTransform3 getTransform();
		HMatrix4 getViewMatrix();
		HMatrix4 getProjectMatrix();
		HMatrix4 getViewProjectMatrix();
		HReal getSpeed() const { return mSpeed; }
		HReal getAspectRatio() const { return mAspectRatio; }
		HReal getNearClip() const { return mNearClip; }
		HReal getFarClip() const { return mFarClip; }
		HReal getFOV() const { return mFOV; }

		void setPose(const HVector3 &eye, const HVector3 &dir);
		void setSpeed(MathLib::HReal speed);

	private:
		void _UpdateMatrix();

	private:
		HVector3 mEye;
		HVector3 mDir;
		MathLib::HReal mAspectRatio;
		MathLib::HReal mNearClip = 1.0f;
		MathLib::HReal mFarClip = 10000.0f;
		MathLib::HReal mFOV = 60.0f;
		int mMouseX;
		int mMouseY;
		MathLib::HReal mSpeed;
		MathLib::HMatrix4 mViewMatrix;
		MathLib::HMatrix4 mProjectMatrix;
		MathLib::HMatrix4 mViewProjectMatrix;
	};

}
