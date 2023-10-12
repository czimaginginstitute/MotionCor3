#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

class CDeviceArray2D
{
public:
	CDeviceArray2D(void);
	~CDeviceArray2D(void);

	void SetInt(void);
	void SetUInt(void);
	void SetFloat(void);

	void SetChannels(int iChannels);	// maximum 2

	void Create(int iSizeX, int iSizeY);
	void Free(void);

	//------------------------------------------------
	// One or two channels. Data must be packed for
	// two channels prior calling this method.
	//------------------------------------------------
	void ToDevice(float* pfData, cudaStream_t stream=0);
	void ToDevice(int* piData, cudaStream_t stream=0);
	void ToDevice(unsigned int* puiData, cudaStream_t stream=0);

	//------------------------------------------------
	// Two channels only, for convenience.
	//------------------------------------------------
	void ToDevice(float* pfData1, float* pfData2);
	void ToDevice(int* piData1, int* piData2);
	void ToDevice(unsigned int* puiData1, unsigned int* puiData2);
	void ToDevice(cudaPitchedPtr aPitchedPtr);

	cudaArray* m_pCudaArray;

private:
	int m_aiSize[2];
	int m_iChannels;
	cudaChannelFormatKind m_aFormat;
};

