#include "CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

//-------------------------------------------------------------------
// 1. Flip image upside down around x (horizontal) axis.
//-------------------------------------------------------------------
static __global__ void mGVertical
(	float* gfImg,
	int iSizeX,
	int iSizeY
)
{       int x = blockIdx.x * blockDim.x + threadIdx.x;
        if(x >= iSizeX) return;
        int i = blockIdx.y * iSizeX + x;
        //------------------------------
	int iY = iSizeY - 1 - blockIdx.y;
	int j = iY * iSizeX + x;
	float fTemp = gfImg[i];
	gfImg[i] = gfImg[j];
	gfImg[j] = fTemp;
}

static __global__ void mGHorizontal
(	float* gfImg,
	int iSizeX,
	int iSizeY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y > iSizeY) return;
	int i = y * iSizeX + blockIdx.x;
	//------------------------------
	int iX = iSizeX - 1 - blockIdx.x;
	int j = y * iSizeX + iX;
	float fTemp = gfImg[i];
	gfImg[i] = gfImg[j];
	gfImg[j] = fTemp;
}

GFlip2D::GFlip2D(void)
{
}

GFlip2D::~GFlip2D(void)
{
}

void GFlip2D::Vertical
(	float* pfImg,
	bool bGpu,
	int* piImgSize
)
{	if(bGpu)
	{	mVertical(pfImg, piImgSize);
		return;
	}
	//-------------
	float* gfImg = mCopyToDevice(pfImg, piImgSize);
	mVertical(gfImg, piImgSize);
	//--------------------------
	size_t tBytes = piImgSize[0] * piImgSize[1] * sizeof(float);
	cudaMemcpy(pfImg, gfImg, tBytes, cudaMemcpyDeviceToHost);
	if(gfImg != 0L) cudaFree(gfImg);
}	

void GFlip2D::Horizontal
(	float* pfImg,
	bool bGpu,
	int* piImgSize
)
{	if(bGpu)
	{	mHorizontal(pfImg, piImgSize);
		return;
	}
	//-------------
	float* gfImg = mCopyToDevice(pfImg, piImgSize);
	mHorizontal(gfImg, piImgSize);
	//----------------------------
	size_t tBytes = piImgSize[0] * piImgSize[1] * sizeof(float);
        cudaMemcpy(pfImg, gfImg, tBytes, cudaMemcpyDeviceToHost);
        if(gfImg != 0L) cudaFree(gfImg);
}

float* GFlip2D::mCopyToDevice(float* pfImg, int* piImgSize)
{
	float* gfImg = 0L;
	size_t tBytes = piImgSize[0] * piImgSize[1] * sizeof(float);
	cudaMalloc(&gfImg, tBytes);
	cudaMemcpy(gfImg, pfImg, tBytes, cudaMemcpyHostToDevice);
	return gfImg;
}

void GFlip2D::mVertical
(	float* gfImg,
	int* piImgSize
)
{	int iHalfY = piImgSize[1] / 2;
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(1, iHalfY);
	aGridDim.x = piImgSize[0] / aBlockDim.x + 1;
	mGVertical<<<aGridDim, aBlockDim>>>
	(  gfImg, piImgSize[0], piImgSize[1]
	);
}

void GFlip2D::mHorizontal
(	float* gfImg,
	int* piImgSize
)
{	int iHalfX = piImgSize[0] / 2;
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(iHalfX, 1);
	aGridDim.y = piImgSize[1] / aBlockDim.y + 1;
	mGHorizontal<<<aGridDim, aBlockDim>>>
	(  gfImg, piImgSize[0], piImgSize[1]
	);
}

