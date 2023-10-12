#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGPositivity2D
(	float* gfImg, 
	int iSizeX
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	int i = blockIdx.y * iSizeX + x;
	gfImg[i] = fmax(0.0f, gfImg[i]);
}

static __global__ void mGAddVal(float* gfImg, int iPixels, float fVal)
{
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= iPixels) return;
	gfImg[i] += fVal;
}

GPositivity2D::GPositivity2D(void)
{
}

GPositivity2D::~GPositivity2D(void)
{
}

void GPositivity2D::DoIt(float* gfImg, int* piImgSize, cudaStream_t stream)
{
	if(gfImg == 0L) return;
	//---------------------
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(piImgSize[0] / aBlockDim.x + 1, piImgSize[1]);
	mGPositivity2D<<<aGridDim, aBlockDim, 0, stream>>>(gfImg, piImgSize[0]);
}

void GPositivity2D::AddVal(float* gfImg, int* piImgSize, float fVal,
	cudaStream_t stream)
{
	if(gfImg == 0L) return;
	int iPixels = piImgSize[0] * piImgSize[1];
	dim3 aBlockDim(512, 1);
	dim3 aGridDim( (iPixels + aBlockDim.x - 1) / aBlockDim.x, 1);
	mGAddVal<<<aGridDim, aBlockDim, 0, stream>>>(gfImg, iPixels, fVal);	
}
