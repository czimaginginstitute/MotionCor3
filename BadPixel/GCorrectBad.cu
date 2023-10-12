#include "CCorrectMain.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::BadPixel;

//-------------------------------------------------------------------
// 1. gfFrame could be padded. In this case, iPadX = (iSizeX/2+1)*2.
//    Otherwise iPadX = iFrameX.
// 2. gucBad is also padded if gfFrame is padded.
//------------------------------------------------------------------- 
static __device__ bool mGFindGood
(	int iFrameX, 
	int iPadX,
	int iWinSize,
	unsigned int next,
	unsigned char* gucBad,
	float* gfFrame
)
{	int iX = 0, iY = 0;
	int iWinSize2 = iWinSize * iWinSize;
	int iHalfWin = iWinSize / 2;
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	for(int k=0; k<500; k++)
	{	next *= 997;
		iX = next % iWinSize2;
		iY = iX / iWinSize - iHalfWin + blockIdx.y;
		if(iY < 0 || iY >= gridDim.y) continue;
		//-------------------------------------
		iX = iX % iWinSize - iHalfWin + x;
		if(iX < 0 || iX >= iFrameX) continue;
		//-----------------------------------
		iX += (iY * iPadX);
		if(gucBad[iX] == 1) continue;
		gfFrame[blockIdx.y * iPadX + x] = gfFrame[iX];
		return true;
	}
	return false;
} 

static __global__ void mGCorrect
(	float* gfFrame,
	int iFrameX,
	int iPadX,
	unsigned char* gucBad,
	int iWinSize
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iFrameX) return;
	int i = blockIdx.y * iPadX + x;
	if(gucBad[i] == 0) return;
	//------------------------
	unsigned int next = i * 109 + 619;
	for(int j=0; j<10; j++)
	{	bool bFind = mGFindGood(iFrameX, iPadX, iWinSize, 
	   	   next, gucBad, gfFrame);
		if(bFind) return;
		iWinSize += 50;
		next *= 997;
	}
	//--------------------------------	
	int iX = next % (iFrameX * gridDim.y);
	int iY = iX / iFrameX;
	iX = iX % iFrameX;
	gfFrame[i] = gfFrame[iY * iPadX + iX];
}

GCorrectBad::GCorrectBad(void)
{
	m_iWinSize = 31;
}

GCorrectBad::~GCorrectBad(void)
{
}

void GCorrectBad::SetWinSize(int iSize)
{
	if(iSize <= m_iWinSize) return;
	m_iWinSize = iSize;
}

void GCorrectBad::GDoIt
(	float* gfFrame,
	unsigned char* gucBadMap,
	int* piFrmSize,
	bool bPadded,
        cudaStream_t stream
)
{	int iFrameX = piFrmSize[0];
	int iPadX = iFrameX;
	if(bPadded) iFrameX = (iPadX / 2 - 1) * 2;
	//----------------------------------------
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(1, piFrmSize[1]);
	aGridDim.x = iFrameX / aBlockDim.x;
	if((iFrameX % aBlockDim.x) > 0) aGridDim.x += 1;
	//----------------------------------------------
	mGCorrect<<<aGridDim, aBlockDim, 0, stream>>>(gfFrame, iFrameX, iPadX, 
	   gucBadMap, m_iWinSize);
}

