#include "CMotionDeconInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2;
using namespace MotionCor2::MotionDecon;

static __device__ float mGCalcDirSinc
(	int iCmpY,
	float fShiftX,
	float fShiftY 
)
{	float fShift = sqrtf(fShiftX * fShiftX + fShiftY * fShiftY);
	if(fShift < 0.1f) return 1.0f;
	//----------------------------
	// slope of the line perpendicular to the motion vector
	//-----------------------------------------------------
	float fSlope = -fShiftX / (fShiftY + (float)1e-20);
	//-------------------------------------------------
	float fDist = 0.0f;
	if(fabs(fSlope) > (float)1e5)
	{	fDist = blockIdx.x;
	}
	else
	{	int y = blockIdx.y * blockDim.y + threadIdx.y;
		if(y > (iCmpY / 2)) y -= iCmpY;
		fDist = fabs(fSlope * blockIdx.x - y) 
			/ sqrtf(fSlope * fSlope + 1.0f);
	}
	if(fDist < 0.1f) return 1.0f;
	//---------------------------
	float fPI = 3.141592654f;
	fDist = fDist / (2 * (gridDim.x - 1));
	if((fShift * fDist) > 1.0f) return 0.0f;
	float fSinc = sinf(fPI * fShift * fDist) / (fPI * fShift * fDist);
	return fSinc;
}

static __device__ float mGCalcSinc
(	int iCmpY,
	float fShift
)
{	fShift += 1.0f;
	//-------------
	float fX = blockIdx.x * 0.5f / (gridDim.x - 1.0f);
	float fY = blockIdx.y * blockDim.y + threadIdx.y;
	fY = fY * 1.0f / iCmpY;
	if(fY > 0.5f) fY -= 1.0f;
	float fR = sqrtf(fX * fX + fY * fY);
	if(fR < 0.0001f) return 1.0f;
	//---------------------------
	float fPI = 3.141592654f;
	float fFact = fShift * fR;
	if(fFact >= 1.0) return 0.0f;
	float fSinc = sinf(fPI * fFact) / (fPI * fFact);
	return fSinc;
}

static __global__ void mGDirWeightFrame
(	float fShiftX,
	float fShiftY,
	cufftComplex* gCmpFrm,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int i = y * gridDim.x + blockIdx.x;
	if(y >= iCmpY || i == 0) return;
	//------------------------------
	float fSinc = mGCalcDirSinc(iCmpY, fShiftX, fShiftY);
	gCmpFrm[i].x *= fSinc;
	gCmpFrm[i].y *= fSinc;
}

static __global__ void mGWeightFrame
(	float fShift,
	cufftComplex* gCmpFrm,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int i = y * gridDim.x + blockIdx.x;
	if(y >= iCmpY || i == 0) return;
	//------------------------------
	float fSinc = mGCalcSinc(iCmpY, fShift);
	gCmpFrm[i].x *= fSinc;
	gCmpFrm[i].y *= fSinc;
} 

GMotionWeight::GMotionWeight(void)
{	
}

GMotionWeight::~GMotionWeight(void)
{
}

void GMotionWeight::DirWeight
(	float* pfMotion,
	cufftComplex* gCmpFrm,
	int* piCmpSize,
        cudaStream_t stream
)
{	double dMotion = sqrtf(pfMotion[0] * pfMotion[0]
		+ pfMotion[1] * pfMotion[1]);
	if(dMotion < 0.1) return;
	//-----------------------
	dim3 aBlockDim(1, 64);
        dim3 aGridDim(piCmpSize[0], 1);
        aGridDim.y = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	//----------------------------------------------------------
	mGDirWeightFrame<<<aGridDim, aBlockDim, 0, stream>>>
	( pfMotion[0], pfMotion[1], 
	  gCmpFrm, piCmpSize[1]
	);
}

void GMotionWeight::Weight
(	float fMotion,
	cufftComplex* gCmpFrm,
	int* piCmpSize,
        cudaStream_t stream
)
{	if(fabs(fMotion) < 0.1) return;
        //-----------------------------
	dim3 aBlockDim(1, 64);
        dim3 aGridDim(piCmpSize[0], 1);
        aGridDim.y = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
        //----------------------------------------------------------
	mGWeightFrame<<<aGridDim, aBlockDim, 0, stream>>>
        ( fMotion, gCmpFrm, piCmpSize[1]
        );
}

