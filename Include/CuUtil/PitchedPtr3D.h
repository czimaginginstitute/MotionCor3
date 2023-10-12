#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

class CPitchedPtr3D
{
public:

	CPitchedPtr3D(void);

	~CPitchedPtr3D(void);

	void Create(int* piSize);

	void Create(int iSizeX, int iSizeY, int iSizeZ);

	void Free(void);

	void ToHost(float* pfData);

	void ToHost(int* piData);

	void ToDevice(float* pfData);

	cudaPitchedPtr m_aPitchedPtr;

	int m_iSize[3];
};

