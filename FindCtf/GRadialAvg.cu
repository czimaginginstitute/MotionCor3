#include "CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

using namespace MotionCor2::FindCtf;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
static __global__ void mGRadAverage
(	float* gfSpectrum,
	float* gfAverage,
	int iCmpX,
	int iCmpY
)
{	int r = blockIdx.x * blockDim.x + threadIdx.x;
	if(r >= iCmpX) return;
	//--------------------
	int iPoints = (int)(3.1415926f * r + 0.5f);
	if(iPoints < 1) iPoints = 1;
	else if(iPoints > 180) iPoints = 180;
	float fSum = 0.0f;
	float fAngStep = 180.0f / iPoints;
	//--------------------------------
	for(int i=0; i<iPoints; i++)
	{	float fRad = (-90.0f + i * fAngStep) * 0.017453f;
		int x = (int)(r * cosf(fRad) + 0.5f);
		int y = (int)(r * sinf(fRad) + iCmpY / 2 + 0.5f);
		fSum += gfSpectrum[y * iCmpX + x];
	}	
	gfAverage[r] = fSum / iPoints;
}

GRadialAvg::GRadialAvg(void)
{
}

GRadialAvg::~GRadialAvg(void)
{
}

void GRadialAvg::DoIt
(	float* gfSpect, 
	float* gfAverage,
	int* piCmpSize
)
{	dim3 aBlockDim(512, 1);
        dim3 aGridDim(1, 1);
	aGridDim.x = (piCmpSize[0] + aBlockDim.x - 1) / aBlockDim.x;
        mGRadAverage<<<aGridDim, aBlockDim>>>(gfSpect, gfAverage,
	   piCmpSize[0], piCmpSize[1]);
}

