#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGCorrectCmp
( 	cufftComplex* gCmpFrm,
	int iCmpSizeY
)
{	int iY = blockIdx.y * blockDim.y + threadIdx.y;
	if(iY >= iCmpSizeY) return;
	if(blockIdx.x == 0 && iY == 0) return;
	int i = iY * gridDim.x + blockIdx.x;
	//----------------------------------
	float fX = blockIdx.x * 0.5f / (gridDim.x - 1);
	float fY = iY / (float)iCmpSizeY;
	if(fY > 0.5f) fY -= 1.0f;
	//-----------------------
	fX = sqrtf(fX * fX + fY * fY) * 3.141592654f;
	fX = sinf(fX) / fX;
	fX = powf(fX, 0.125f);
	//--------------------
	gCmpFrm[i].x /= fX;
	gCmpFrm[i].y /= fX;
}

GCorrLinearInterp::GCorrLinearInterp(void)
{
}

GCorrLinearInterp::~GCorrLinearInterp(void)
{
}

void GCorrLinearInterp::DoIt
( 	cufftComplex* gCmpFrm, 
	int* piCmpSize,
        cudaStream_t stream
)
{	dim3 aBlockDim(1, 512);
	int iGridY = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	dim3 aGridDim(piCmpSize[0], iGridY);
	mGCorrectCmp<<<aGridDim, aBlockDim, 0, stream>>>(gCmpFrm, piCmpSize[1]);
}

