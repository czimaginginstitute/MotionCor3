#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGThreshold2D
(	float* gfImg,
	float fMin,
	float fMax,
	int iPadX,
	int iSizeY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iSizeY) return;
	int i = y * iPadX + blockIdx.x;
	float fInt = gfImg[i];
	if(fInt <= (float)-1e20) return;
	else if(gfImg[i] < fMin) gfImg[i] = fMin;
	else if(gfImg[i] > fMax) gfImg[i] = fMax;
}

GThreshold2D::GThreshold2D(void)
{
}

GThreshold2D::~GThreshold2D(void)
{
}

void GThreshold2D::DoIt
(	float* gfImg, int* piImgSize, bool bPadded,
	float fMin, float fMax
)
{	int iSizeX = piImgSize[0];
	if(bPadded) iSizeX = (piImgSize[0] / 2 - 1) * 2;
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(iSizeX, 1);
	aGridDim.y = (piImgSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	mGThreshold2D<<<aGridDim, aBlockDim>>>(gfImg, fMin, fMax,
	   piImgSize[0], piImgSize[1]);
}
