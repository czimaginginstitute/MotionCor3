#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

class CDeviceData
{
public:

	CDeviceData(void);

	~CDeviceData(void);

	void Create(unsigned int iBytes);

	void Clear(void);

	void ToDevice(short* psData, int iSize);

	void ToDevice(int* piData, int iSize);

	void ToDevice(float* pfData, int iSize);

	void SetDevice(void* pvData, unsigned int iNumBytes);

	void SetHost(void* pvData, unsigned int iNumBytes);

	void ToHost(short* psData, int iSize);

	void ToHost(int* piData, int iSize);

	void ToHost(float* pfData, int iSize);

	void* m_pvData;

	unsigned int m_iBytes;
};

