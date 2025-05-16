#pragma once
#include <string>
#include <vector>
#include <CL/cl.h>

class OCLAcceleration
{
public:
	OCLAcceleration(void) {};
	~OCLAcceleration(void) {};
	bool InitPlatform(const unsigned int platformIndex = 0);
	bool InitDevice(const unsigned int deviceIndex);
	bool GetPlatformsInfo(std::vector<std::string>& info, const std::string& indentation);
	bool GetDevicesInfo(std::vector<std::string>& info, const std::string& indentation);
	cl_platform_id* GetPlatform() { return &m_platform; }
	const cl_platform_id* GetPlatform() const { return &m_platform; }
	cl_device_id* GetDevice() { return &m_device; }
	const cl_device_id* GetDevice() const { return &m_device; }

private:
	cl_platform_id m_platform = nullptr;
	cl_device_id m_device = nullptr;
	cl_int m_lastError = 0;
};

inline bool InitOCL(const unsigned int oclPlatformID, const unsigned int oclDeviceID, OCLAcceleration &oclHelper,const bool printInfo=false)
{
	bool res = true;
	std::vector<std::string> info;
	res = oclHelper.GetPlatformsInfo(info, "\t\t");
	if (!res)
		return res;

	if (printInfo)
	{
		const size_t numPlatforms = info.size();
		printf("\t Number of OpenCL platforms:%d \n", numPlatforms);
		for (size_t i = 0; i < numPlatforms; ++i)
		{
			printf("\t OpenCL platform [%d]", i);
			printf(info[i].c_str());
		}
		printf("\t Using OpenCL platform [%d]", oclPlatformID);
	}
	res = oclHelper.InitPlatform(oclPlatformID);
	if (!res)
		return res;

	info.clear();
	res = oclHelper.GetDevicesInfo(info, "\t\t");
	if (!res)
		return res;
	if (printInfo)
	{
		const size_t numDevices = info.size();
		printf("\t Number of devices:%d \n", numDevices);
		for (size_t i = 0; i < numDevices; ++i)
		{
			printf("\t Opencl device [%d]", i);
			printf(info[i].c_str());
		}
		printf("\t Using OpenCL device [%d]", oclDeviceID);
	}
	res = oclHelper.InitDevice(oclDeviceID);
	return res;
}