#pragma once
#include "Physics/PhysicsTypes.h"
namespace MathLib
{
	class Camera
	{
	public:
		Camera(const HVector3 &eye, const HVector3 &dir);

		void handleMouse(int button, int state, int x, int y);
		bool handleKey(unsigned char key, int x, int y, float speed = 0.5f);
		void handleMotion(int x, int y);
		void handleAnalogMove(float x, float y);

		HVector3 getEye() const;
		HVector3 getDir() const;
		HTransform3 getTransform() const;

		void setPose(const HVector3 &eye, const HVector3 &dir);
		void setSpeed(float speed);

	private:
		HVector3 mEye;
		HVector3 mDir;
		int mMouseX;
		int mMouseY;
		float mSpeed;
	};

}
