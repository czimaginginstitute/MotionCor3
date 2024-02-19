#include "Util_Time.h"
#include <sys/time.h>
#include <stdio.h>

void Util_Time::Measure(void)
{
     gettimeofday(&m_aTimeval, NULL);
}

float Util_Time::GetElapsedSeconds(void)
{
	struct timeval tp;
	gettimeofday(&tp, NULL);
	double dSeconds = tp.tv_sec - m_aTimeval.tv_sec
		+ (tp.tv_usec - m_aTimeval.tv_usec) * 1e-6;
	return (float)dSeconds;
}   	
