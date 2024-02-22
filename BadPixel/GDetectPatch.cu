#include "CDetectMain.h"
#include "../CMainInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>

using namespace MotionCor2::BadPixel;

static __global__ void mGThreshold
(	float* gfPadCC,
	int iSizeX,
	int iPadX,
	float fThreshold
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	//---------------------
	int i = blockIdx.y * iPadX + x;
	if(gfPadCC[i] <= fThreshold) gfPadCC[i] = 0;
	else gfPadCC[i] = 1;
}

static __global__ void mGUpdateBadMap
(	float* gfPadCC,
	unsigned char* gucBadMap,
	int iSizeX,
	int iPadX,
	int iModSizeX,
	int iModSizeY
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if((x + iModSizeX) >= iSizeX) return;
	if((blockIdx.y + iModSizeY) >= gridDim.y) return;
	//-----------------------------------------------
	int i = blockIdx.y * iPadX + x;
	if(gfPadCC[i] == 0) return;
	//-------------------------
	int iOffset = blockIdx.y * iPadX + x;
	for(int y=0; y<iModSizeY; y++)
	{	i = iOffset + y * iPadX;
		for(x=0; x<iModSizeX; x++)
		{	gucBadMap[i+x] = 1;
		}
	}
}

GDetectPatch::GDetectPatch(void)
{
}

GDetectPatch::~GDetectPatch(void)
{
}

void GDetectPatch::DoIt
(  	float* gfPadSum,
	float* gfPadCC,
	float* gfPadBuf,
	unsigned char* pucBadMap,
	int* piPadSize, 
	int* piModSize,
	float fStdThreshold
)
{	CLocalCCMap localCCMap;
	localCCMap.DoIt(piModSize);
	//-----------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(0);
	//-----------------
	Util::GCalcMoment2D calcMoment;
	calcMoment.SetSize(piPadSize, true);
	float fMean = calcMoment.DoIt(gfPadCC, 1, true);
	float fStd = calcMoment.DoIt(gfPadCC, 2, true);
	fStd = (float)sqrtf(fmaxf(0, fStd - fMean * fMean));
	//-----------------
	m_fCCThreshold = fMean + fStdThreshold * fStd;
	printf("CC Mean Std Threshold: %.3e  %.3e  %.3f\n\n",
	   fMean, fStd, m_fCCThreshold);
	//-----------------
	mUpdateBadMap(gfPadCC, pucBadMap, piPadSize, piModSize);
}

void GDetectPatch::mUpdateBadMap
(	float* gfPadCC, 
	unsigned char* pucBadMap,
	int* piPadSize,
	int* piModSize
)
{	int iImgSizeX = (piPadSize[0] / 2 - 1) * 2;
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(1, piPadSize[1]);
	aGridDim.x = (iImgSizeX + aBlockDim.x - 1) / aBlockDim.x;
	//-------------------------------------------------------
	// convert gfCCMap to binary map by thresholding.
	//-------------------------------------------------------
	mGThreshold<<<aGridDim, aBlockDim>>>(gfPadCC, iImgSizeX,
		piPadSize[0], m_fCCThreshold);
	//------------------------------------
	// update the existing bad pixel map
	//------------------------------------
	mGUpdateBadMap<<<aGridDim, aBlockDim>>>(gfPadCC, pucBadMap,
		iImgSizeX, piPadSize[0], piModSize[0], piModSize[1]);
}

