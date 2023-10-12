#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

class CDeviceInfo
{
public:

	CDeviceInfo(void);

	~CDeviceInfo(void);

	void Query(void);

	//----------------------
	// return number of GPUs
	// installed.
	//----------------------
	int GetNumDevices(void);

	//-----------------------
	// return total memory
	// of all GPUs in MB.
	//-----------------------
	int GetTotalMemory(void);

	int GetDeviceMemory(int iDeviceID);

private:

	int m_iNumDevices;

	int* m_piMemory;

	int m_iTotalMemory;
};
