#include "CUtilInc.h"
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2::Util;

template <typename T>
static __global__ void mGPad(T* pSrc, int iSizeX, T* pDst)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if (x >= iSizeX) return;
	//----------------------
	int iPadX = (iSizeX / 2 + 1) * 2;
	pDst[blockIdx.y * iPadX + x] = pSrc[blockIdx.y * iSizeX + x];
}

template <typename T>
static __global__ void mGUnpad(T* pSrc, int iSizeX, T* pDst)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	//---------------------
	int iPadX = (iSizeX / 2 + 1) * 2;
	pDst[blockIdx.y * iSizeX + x] = pSrc[blockIdx.y * iPadX + x];
}

void GPad2D::Pad
(	float* gfImg,
	int* piImgSize,
	float* gfPadImg,
	cudaStream_t stream
)
{	dim3 aBlockDim (64, 1, 1);
        dim3 aGridDim (piImgSize[0] / aBlockDim.x + 1, piImgSize[1]);
        mGPad<<<aGridDim,aBlockDim, 0, stream>>>(gfImg, 
		piImgSize[0], gfPadImg);
}

void GPad2D::Unpad
(	float* gfPadImg,
	int* piPadSize,
	float* gfImg,
	cudaStream_t stream
)
{	int iImgSizeX = (piPadSize[0] / 2 - 1) * 2;
	dim3 aBlockDim(64, 1, 1);
	dim3 aGridDim(iImgSizeX / aBlockDim.x + 1, piPadSize[1]);
	mGUnpad<<<aGridDim, aBlockDim, 0, stream>>>(gfPadImg, 
		iImgSizeX, gfImg);
}
