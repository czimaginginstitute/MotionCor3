#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGNorm2D
(	float* gfImg,
	int iPadX,
	int iSizeY,
	float fMean,
	float fStd
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iSizeY) return;
	//---------------------
	int i = y * iPadX + blockIdx.x;
	float fInt = gfImg[i];
	if(fInt < (float)-1e10) return;
	//-----------------------------
	gfImg[i] = (fInt - fMean) / fStd;
}


GNormalize2D::GNormalize2D(void)
{
}

GNormalize2D::~GNormalize2D(void)
{
}

void GNormalize2D::DoIt
(	float* gfImg,
	int* piSize,
	bool bPadded,
	float fMean,
	float fStd,
	cudaStream_t stream
)
{	int iSizeX = bPadded ? (piSize[0] / 2 - 1) * 2 : piSize[0];
	int iSizeY = piSize[1];
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(iSizeX, 1);
	aGridDim.y = (iSizeY + aBlockDim.y - 1) / aBlockDim.y;
	mGNorm2D<<<aGridDim, aBlockDim, 0, stream>>>
	( gfImg, piSize[0], iSizeY, fMean, fStd
	);
}

