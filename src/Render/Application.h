#pragma once

class PhysicsEngineTestingApplication
{
public:
	virtual void Release() = 0;
	virtual int Run() = 0;
};

extern PhysicsEngineTestingApplication* CreatePhysicsEngineTestingApplication(int argc, char** argv);
extern PhysicsEngineTestingApplication* GetPhysicsEngineTestingApplication();