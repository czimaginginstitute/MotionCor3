#pragma once

#include <sys/time.h>

class Util_Time
{
public:

	void Measure(void);

	float GetElapsedSeconds(void);

private:

	struct timeval m_aTimeval;

};

