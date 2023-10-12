#include "CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::FindCtf;

static const int BLOCK_SIZE_X = 512;

//-------------------------------------------------------------------
// giImgSize[0]: image size in x, unpadded size in x.
// giImgSize[1]: image pixels = giImgSize[0] * iImgSizeY
// iPadX:        If gfImg is padded in x, iPadX = (giImgSize[0] 
//               / 2 + 1) * 2. 
//               If gfImg is not padded, iPadX = giImgSize[0].
// gfImg:        It can be either original image or padded image
//               for FFT. In the latter case, iPadX is the padded
//               size in x and giImgSize[0] is still the orginal
//               image size in x.
//-------------------------------------------------------------------
__device__ __constant__ int giImgSize[2];

static __global__ void mGCalcSum
(	float* gfImg,
	int iPadX,
	float* gfSum,
	int* giCount
)
{	__shared__ float sfSum[BLOCK_SIZE_X];
	__shared__ int siCount[BLOCK_SIZE_X];
	sfSum[threadIdx.x] = 0;
	siCount[threadIdx.x] = 0;
	//-----------------------
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= giImgSize[1]) return;
	i = (i / giImgSize[0]) * iPadX + (i % giImgSize[0]);
	//--------------------------------------------------
	float fInt = gfImg[i];
	if(fInt > (float)-1e10)
	{	sfSum[threadIdx.x] = fInt;
		siCount[threadIdx.x] = 1;
	}
	__syncthreads();
	//--------------
	i = BLOCK_SIZE_X / 2;
	while(i > 0)
	{	if(threadIdx.x < i)
		{	sfSum[threadIdx.x] += sfSum[i+threadIdx.x];
			siCount[threadIdx.x] += siCount[i+threadIdx.x];
		}
		__syncthreads();
		i /= 2;
	}
	if(threadIdx.x == 0)
	{	gfSum[blockIdx.x] = sfSum[0];
		giCount[blockIdx.x] = giCount[0];
	}
}

static __global__ void mGRemoveMean
(	int iPadX,
	float fMean,
	float* gfImg
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= giImgSize[1]) return;
	i = (i / giImgSize[0]) * iPadX + (i % giImgSize[0]);
	//--------------------------------------------------
	float fInt = gfImg[i];
	if(fInt > (float)-1e10) gfImg[i] = fInt - fMean;
	else gfImg[i] = 0.0f;
}


GRemoveMean::GRemoveMean(void)
{
	m_iPadX = 0;
	memset(m_aiImgSize, 0, sizeof(m_aiImgSize));
}

GRemoveMean::~GRemoveMean(void)
{
}

//-------------------------------------------------------------------
// 1. When the image is not padded, m_iPadX = piImgSize[0].
//-------------------------------------------------------------------
void GRemoveMean::DoIt
(	float* pfImg,
	bool bGpu,
	int* piImgSize
)
{	float* gfImg = pfImg;
	if(!bGpu) gfImg = mToDevice(pfImg, piImgSize);
	m_iPadX = piImgSize[0];
	memcpy(m_aiImgSize, piImgSize, sizeof(m_aiImgSize));
	//--------------------------------------------------
	float fMean = mCalcMean(gfImg);
	mRemoveMean(gfImg, fMean);
	//------------------------
	if(bGpu) return;
	int iPixels = m_aiImgSize[0] * m_aiImgSize[1];
	size_t tBytes = sizeof(float) * iPixels;
	cudaMemcpy(pfImg, gfImg, tBytes, cudaMemcpyDeviceToHost);
	if(gfImg != 0L) cudaFree(gfImg);
}

//-------------------------------------------------------------------
// 1. When the image is padded for FFT, m_iPadX = piPadSize[0].
// 2. m_aiImgSize[0] = (piPadSize[0] / 2 - 1) * 2;
// 3. Number of cuda threads are determined based upon unpadded
//    image pixels. The padded (extra pixels are not taken into
//    account during the calculation.
//------------------------------------------------------------------- 
void GRemoveMean::DoPad
(	float* pfPadImg,
	bool bGpu,
	int* piPadSize
)
{	float* gfPadImg = pfPadImg;
	if(!bGpu) gfPadImg = mToDevice(pfPadImg, piPadSize);
	m_iPadX = piPadSize[0];
	m_aiImgSize[0] = (piPadSize[0] / 2 - 1) * 2;
	m_aiImgSize[1] = piPadSize[1];
	//----------------------------
	float fMean = mCalcMean(gfPadImg);
	mRemoveMean(gfPadImg, fMean);
	//---------------------------
	if(bGpu) return;
	int iPadSize = piPadSize[0] * piPadSize[1];
	size_t tBytes = sizeof(float) * iPadSize;
	cudaMemcpy(pfPadImg, gfPadImg, tBytes, cudaMemcpyDeviceToHost);
	if(gfPadImg != 0L) cudaFree(gfPadImg);
}	

float* GRemoveMean::mToDevice(float* pfImg, int* piSize)
{
	size_t tBytes = sizeof(float) * piSize[0] * piSize[1];
	float* gfBuf = 0L;
	cudaMalloc(&gfBuf, tBytes);
	cudaMemcpy(gfBuf, pfImg, tBytes, cudaMemcpyHostToDevice);
	return gfBuf;
}

float GRemoveMean::mCalcMean(float* gfImg)
{
	int iPixels = m_aiImgSize[0] * m_aiImgSize[1];
	int aiImgSize[] = {m_aiImgSize[0], iPixels};
	cudaMemcpyToSymbol(giImgSize, aiImgSize, sizeof(giImgSize));
	//----------------------------------------------------------
	int iSegments = iPixels / BLOCK_SIZE_X + 1;
	float* gfSum = 0L;
	size_t tBytes = iSegments * sizeof(float);
	cudaMalloc(&gfSum, tBytes);
	cudaMemset(gfSum, 0, tBytes);
	//---------------------------
	int* giCount = 0L;
	tBytes = iSegments * sizeof(int);
	cudaMalloc(&giCount, tBytes);
	cudaMemset(giCount, 0, tBytes);
	//-----------------------------
	dim3 aBlockDim(BLOCK_SIZE_X, 1);
	dim3 aGridDim(iSegments, 1);
	mGCalcSum<<<aGridDim, aBlockDim>>>
	(  gfImg, m_iPadX, 
	   gfSum, giCount
	);
	//---------------
	cudaMemcpyKind aD2H = cudaMemcpyDeviceToHost;
	float* pfSum = new float[iSegments];
	int* piCount = new int[iSegments];
	cudaMemcpy(pfSum, gfSum, iSegments * sizeof(float), aD2H);
	cudaMemcpy(piCount, giCount, iSegments * sizeof(int), aD2H);
	if(gfSum != 0L) cudaFree(gfSum);
	if(giCount != 0L) cudaFree(giCount);
	//----------------------------------
	double dSum = 0;
	int iCount = 0;
	for(int i=0; i<iSegments; i++)
	{	dSum += pfSum[i];
		iCount += piCount[i];
	}
	if(pfSum != 0L) delete[] pfSum;
	if(piCount != 0L) delete[] piCount;
	//---------------------------------
	float fMean = 0.0f;
	if(iCount > 0) fMean = (float)(dSum / iCount);
	return fMean;	
}

void GRemoveMean::mRemoveMean(float* gfImg, float fMean)
{
	int iPixels = m_aiImgSize[0] * m_aiImgSize[1];
        dim3 aBlockDim(512, 1);
        dim3 aGridDim(1, 1);
        aGridDim.x = iPixels / aBlockDim.x + 1;
        mGRemoveMean<<<aGridDim, aBlockDim>>>
        (  m_iPadX, fMean, gfImg
        );
}
