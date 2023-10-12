#include "CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

//-------------------------------------------------------------------
// 1. Flip image upside down around x (horizontal) axis.
//-------------------------------------------------------------------
static __global__ void mGAugment
(	float* gfInRef,
	int iInSizeX,
	int iFact,
	float* gfOutRef,
	int iOutSizeX,
	unsigned int iOutSize
)
{       unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= iOutSize) return;
        //-----------------------
	unsigned int y = (i / iOutSizeX) / iFact;
	unsigned int j = y * iInSizeX + (i % iOutSizeX) / iFact;
	gfOutRef[i] = gfInRef[j];
}

GAugmentRef::GAugmentRef(void)
{
}

GAugmentRef::~GAugmentRef(void)
{
}

void GAugmentRef::DoIt
(	float* gfInRef,
	int* piInSize,
	float* gfOutRef,
	int* piOutSize
)
{	int iFact = piOutSize[0] / piInSize[0];
	unsigned int iOutSize = piOutSize[0] * piOutSize[1];
	dim3 aBlockDim(512, 1);
	dim3 aGridDim((iOutSize + aBlockDim.x - 1) / aBlockDim.x, 1);
	mGAugment<<<aGridDim, aBlockDim>>>(gfInRef, piInSize[0], iFact,
	   gfOutRef, piOutSize[0], iOutSize);
}

