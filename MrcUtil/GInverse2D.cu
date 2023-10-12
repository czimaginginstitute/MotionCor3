#include "CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

//-------------------------------------------------------------------
// 1. Inverse each pixel value. If the value is zero, it is set to
//    zero. This is for EER reciprocal gain.
//-------------------------------------------------------------------
static __global__ void mGInverse(float* gfImg, int iSizeX)
{	
	int x = blockIdx.x * blockDim.x + threadIdx.x;
        if(x >= iSizeX) return;
        int i = blockIdx.y * iSizeX + x;
        //------------------------------
	float fTemp = gfImg[i];
	if(fTemp == 0) gfImg[i] = 0.0f;
	else gfImg[i] = 1.0f / fTemp;
}

GInverse2D::GInverse2D(void)
{
}

GInverse2D::~GInverse2D(void)
{
}

void GInverse2D::DoIt
(	float* pfImg,
	bool bGpu,
	int* piImgSize
)
{	if(bGpu)
	{	mInverse(pfImg, piImgSize);
		return;
	}
	//-------------
	float* gfImg = 0L;
	size_t tBytes = sizeof(float) * piImgSize[0] * piImgSize[1];
	cudaMalloc(&gfImg, tBytes);
	cudaMemcpy(gfImg, gfImg, tBytes, cudaMemcpyDefault);
	//--------------------------------------------------
	mInverse(gfImg, piImgSize);
	//--------------------------
	cudaMemcpy(pfImg, gfImg, tBytes, cudaMemcpyDefault);
	cudaFree(gfImg);
}	

void GInverse2D::mInverse
(	float* gfImg,
	int* piImgSize
)
{	dim3 aBlockDim(512, 1);
	dim3 aGridDim(1, piImgSize[1]);
	aGridDim.x = (piImgSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	mGInverse<<<aGridDim, aBlockDim>>>(gfImg, piImgSize[0]);
}
