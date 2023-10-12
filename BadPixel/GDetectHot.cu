#include "CDetectMain.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::BadPixel;

static __global__ void mGDetectHot
(	float* gfImg,
	int iPadX,
	int iSizeY,
	float fTol,
	unsigned char* gucBadMap
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iSizeY) return;
	int i = y * iPadX + blockIdx.x;
	if(gfImg[i] <= fTol) gucBadMap[i] = 0;
	else gucBadMap[i] = 1;
}

GDetectHot::GDetectHot(void)
{
	m_iNumHots = 0;
}

GDetectHot::~GDetectHot(void)
{
}

void GDetectHot::DoIt
(	float* gfPadImg,
	float* gfPadBuf, 
	int* piPadSize, 
	float fStdThreshold,
	unsigned char* pucBadMap
)
{	Util::GCalcMoment2D calcMoment;
	calcMoment.SetSize(piPadSize, true);
	float fMean = calcMoment.DoIt(gfPadImg, 1, true);
	float fStd = calcMoment.DoIt(gfPadImg, 2, true);
	fStd = fStd - fMean * fMean;
	fStd = (float)sqrtf(fmaxf(fStd, 0));
	//----------------------------------
	float fDelta = fStdThreshold * fStd;
	if(fDelta < 10) fDelta = 10;
	float fMax = fMean + fDelta;
	//--------------------------
	int iSizeX = (piPadSize[0] / 2 - 1) * 2;
	dim3 aBlockDim(1, 64);
	dim3 aGridDim(iSizeX, 1);
	aGridDim.y = (piPadSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	mGDetectHot<<<aGridDim, aBlockDim>>>
	( gfPadImg, piPadSize[0], piPadSize[1], 
	  fMax, pucBadMap ); 
	//------------------
	printf("Mean & Std: %8.2f %8.2f\n", fMean, fStd);
	printf("Hot pixel threshold: %8.2f\n\n", fMax);
}
