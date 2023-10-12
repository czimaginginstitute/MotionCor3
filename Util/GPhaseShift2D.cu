#include "CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <math.h>

#define OPT_NVIDIA

using namespace MotionCor2::Util;

static __global__ void mGShiftFrame
(	cufftComplex* gInCmp,
	float fShiftX,
	float fShiftY,
	bool bSum,	
	cufftComplex* gOutCmp,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
        if (y >= iCmpY) return;
        int i = y * gridDim.x + blockIdx.x;
	//---------------------------------
	if(y  > (iCmpY / 2)) y -= iCmpY;
        float fPhaseShift = blockIdx.x * fShiftX + y * fShiftY;
	float fCos = cosf(fPhaseShift);
	float fSin = sinf(fPhaseShift);
	//-----------------------------
        cufftComplex res;
	res.x = fCos * gInCmp[i].x - fSin * gInCmp[i].y;
	res.y = fCos * gInCmp[i].y + fSin * gInCmp[i].x;
	//----------------------------------------------
	if(bSum) 
	{	gOutCmp[i].x += res.x;
		gOutCmp[i].y += res.y;
	}
	else gOutCmp[i] = res;
}

GPhaseShift2D::GPhaseShift2D(void)
{
}

GPhaseShift2D::~GPhaseShift2D(void)
{
}

void GPhaseShift2D::DoIt
(	cufftComplex* gInCmp,
	int* piCmpSize,
	float* pfShift,
	bool bSum,
	cufftComplex* gOutCmp,
        cudaStream_t stream
)
{	float f2PI = (float)(8 * atan(1.0));
	int iNx = (piCmpSize[0] - 1) * 2;
	float fShiftX = pfShift[0] * (f2PI / iNx);
	float fShiftY = pfShift[1] * (f2PI / piCmpSize[1]);
        //-------------------------------------------------
	dim3 aBlockDim(1, 64);
	dim3 aGridDim(piCmpSize[0], 1);
	aGridDim.y = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
        mGShiftFrame<<<aGridDim, aBlockDim, 0, stream>>>(gInCmp, 
	   fShiftX, fShiftY, bSum, gOutCmp, piCmpSize[1]);
}

void GPhaseShift2D::DoIt
(	cufftComplex* gCmpFrm,
	int* piCmpSize,
	float* pfShift,
	cudaStream_t stream
)
{	double dShift = sqrtf(pfShift[0] * pfShift[0]
		+ pfShift[1] * pfShift[1]);
	if(dShift < 0.1) return;
	//----------------------
	this->DoIt(gCmpFrm, piCmpSize, pfShift, false, gCmpFrm, stream);
}
