#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

class CDeviceArray3D
{
public:

	CDeviceArray3D(void);

	~CDeviceArray3D(void);

	void Create(int iSizeX, int iSizeY, int iSizeZ);

	void Create(int* piSize);

	void Create(int* piSize, int iNumChannels);

	void Free(void);

	void HostToDevice(float* pfData);

	void DeviceToDevice(float* gfData);

	void DeviceToDevice(cudaPitchedPtr aPitchedPtr);

	cudaArray* m_pCudaArray;

private:

	void mFromHostToDevice(float* pfData);

	void mFromDeviceToDevice(float* gfData);

	int m_aiSize[3];

	int m_iNumChannels;
};

