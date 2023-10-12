#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

class CPitchedPtr2D
{
public:

	CPitchedPtr2D(void);

	~CPitchedPtr2D(void);

	void Create(int* piSize);

	void Create(int iSizeX, int iSizeY);

	void Free(void);

	void Init(int iVal);

	void ToHost(float* pfData);

	cudaPitchedPtr m_aPitchedPtr;

private:

	int m_iSize[2];
};

