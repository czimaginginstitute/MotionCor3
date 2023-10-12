#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

class CDeviceArray1D
{
public:

	CDeviceArray1D(void);

	~CDeviceArray1D(void);

	void Create1(int iSize);
	void Create2(int iSize);
	void Create3(int iSize);
	void Create4(int iSize);
	void Create(int iSize, int iChanels);

	void Free(void);

	void ToDevice1
	(  float* pfData, 
	   bool bFromHost
	);
	void ToDevice2
	(  float* pfData1, float* pfData2, 
	   bool bFromHost
	);

	void ToDevice3
	(  float* pfData1, float* pfData2, 
	   float* pfData3, bool bFromHost
	);

	void ToDevice4
	(  float* pfData1, float* pfData2,
	   float* pfData3, float* pfData4, 
	   bool bFromHost
	);
	void ToDevice
	(  float* pfData,
	   bool bGpu
	);
	cudaArray* m_pCudaArray;

private:
	cudaMemcpyKind mGetCopyKind(bool bFromHost);
	int m_iSize;
	int m_iChannels;
};
