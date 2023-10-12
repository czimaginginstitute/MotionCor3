#include "CUtilInc.h"
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2::Util;

static __global__ void mGAddFloat
(	float* gfFrame1, float fFactor1,
	float* gfFrame2, float fFactor2,
	float* gfSum, int nxy
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= nxy) return;
	gfSum[i] = gfFrame1[i] * fFactor1 + gfFrame2[i] * fFactor2;
}

static __global__ void mGAddComplex
(	cufftComplex* gCmp1, float fFactor1,
	cufftComplex* gCmp2, float fFactor2,
	cufftComplex* gCmpSum, int nxy
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= nxy) return;
        cufftComplex res;
        res.x = gCmp1[i].x * fFactor1 + gCmp2[i].x * fFactor2;
	res.y = gCmp1[i].y * fFactor1 + gCmp2[i].y * fFactor2;
	gCmpSum[i] = res;
}

static __global__ void mGAddUChar
(	unsigned char* gucFrm1,
	unsigned char* gucFrm2,
	unsigned char* gucSum,
	int nxy
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= nxy) return;
	gucSum[i] = gucFrm1[i] + gucFrm2[i];
}

GAddFrames::GAddFrames(void)
{
}

GAddFrames::~GAddFrames(void)
{
}

void GAddFrames::DoIt
(	float* gfFrame1,
	float fFactor1,
	float* gfFrame2,
	float fFactor2,
	float* gfSum,
	int* piFrmSize,
        cudaStream_t stream
)
{       int nxy = piFrmSize[0] * piFrmSize[1];
        dim3 aBlockDim(512, 1, 1);
        dim3 aGridDim((nxy + aBlockDim.x - 1) / aBlockDim.x, 1, 1);
	mGAddFloat<<<aGridDim, aBlockDim, 0, stream>>>
	( gfFrame1, fFactor1,
	  gfFrame2, fFactor2,
	  gfSum, nxy
	);
}

void GAddFrames::DoIt
(	cufftComplex* gCmp1,
	float fFactor1,
	cufftComplex* gCmp2,
	float fFactor2,
	cufftComplex* gCmpSum,
	int* piCmpSize,
        cudaStream_t stream
)
{	int nxy = piCmpSize[0] * piCmpSize[1];
        dim3 aBlockDim(512, 1, 1);
        dim3 aGridDim((nxy + aBlockDim.x - 1) / aBlockDim.x, 1, 1);
	mGAddComplex<<<aGridDim, aBlockDim, 0, stream>>>
	( gCmp1, fFactor1,
	  gCmp2, fFactor2,
	  gCmpSum, nxy
	);
}

void GAddFrames::DoIt
(	unsigned char* gucFrm1,
	unsigned char* gucFrm2,
	unsigned char* gucSum,
	int* piFrmSize,
	cudaStream_t stream
)
{	int nxy = piFrmSize[0] * piFrmSize[1];
        dim3 aBlockDim(128, 1, 1);
        dim3 aGridDim((nxy + aBlockDim.x - 1) / aBlockDim.x, 1, 1);
        mGAddUChar<<<aGridDim, aBlockDim, 0, stream>>>(gucFrm1,
	   gucFrm2, gucSum, nxy);
}
