#pragma once
#include <chrono>

class FrameProfiler
{
public:
	void Start() 
	{
		m_start = std::chrono::steady_clock::now();
	};
	void End()
	{
		m_end = std::chrono::steady_clock::now();
	};
	float GetFrameTime()
	{
		std::chrono::duration<float> duration = m_end - m_start;
		return duration.count();
	};

	float GetFrameRate()
	{
		return 1.0f / GetFrameTime();
	};

private:
	std::chrono::time_point<std::chrono::steady_clock> m_start;
	std::chrono::time_point<std::chrono::steady_clock> m_end;
};