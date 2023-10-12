#include "CDetectMain.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::BadPixel;

static __constant__ int giRefSize[2]; 
static __constant__ int giPadSize[2];

//-------------------------------------------------------------------
// 1. The location of each detected patch is represented by the
//    coordinate of its first pixel.
// 2. Each patch is defined as a square patch.
// 3. gfCC is the cross correlation map that has the same dimension
//    as gfImg. The coordinate of CC peak in the CC map corresponds 
//    to the patch location in the image.
//------------------------------------------------------------------- 
static __global__ void mGLocalCC
(	float* gfPadImg,
	int iOffset,
	int iPartSize,
	float* gfRef, 
	float* gfPadCC
)
{	int iPix = blockIdx.x * blockDim.x + threadIdx.x;
	if(iPix >= iPartSize) return;
	//------------------------
	iPix = iPix + iOffset;
	int x = iPix % giPadSize[0];
	int y = iPix / giPadSize[0];
	iPix = y * giPadSize[0] + x;
	gfPadCC[iPix] = 0.0f;
	//-------------------
	if((x + giRefSize[0]) >= ((giPadSize[0] / 2 - 1) * 2)) return;
	if((y + giRefSize[1]) >= giPadSize[1]) return;
	//--------------------------------------------
	float* gfSrc = gfPadImg + iPix;
	float fMean = 0.0f, fStd = 0.0f, fCC = 0.0f;
	//------------------------------------------
	for(int j=0; j<giRefSize[1]; j++)
	{	for(int i=0; i<giRefSize[0]; i++)
		{	float fImg = gfSrc[j * giPadSize[0] + i];
			float fRef = gfRef[j * giRefSize[0] + i];
			//---------------------------------------
			fMean += fImg;
			fStd += (fImg * fImg);
			fCC += (fRef * fImg);
		}
	}
	int iRefSize = giRefSize[0] * giRefSize[1];
	fCC /= iRefSize;
	fMean /= iRefSize;
	fStd = fStd / iRefSize - fMean * fMean;
	if(fStd < 0) fStd = 0.0f;
	else fStd = sqrtf(fStd);
	//----------------------
	if(fStd == 0) gfPadCC[iPix] = 0.0f;
	else gfPadCC[iPix] = fabsf(fCC) / fStd;
}

GLocalCC::GLocalCC(void)
{
}

GLocalCC::~GLocalCC(void)
{
}

void GLocalCC::SetRef(float* gfRef, int* piRefSize)
{
	m_gfRef = gfRef;
	cudaMemcpyToSymbol(giRefSize, piRefSize, sizeof(int) * 2);
}

void GLocalCC::DoIt
(	float* gfPadImg, 
	int* piPadSize, 
	int iOffset,
	int iPartSize,
	float* gfPadCC,
	cudaStream_t stream
)
{	cudaMemcpyToSymbolAsync
	( giPadSize, piPadSize, sizeof(int) * 2, 0,
	  cudaMemcpyHostToDevice, stream );
	//---------------------------------
	dim3 aBlockDim(64, 1);
        dim3 aGridDim(1, 1);
	aGridDim.x = (iPartSize + aBlockDim.x - 1) / aBlockDim.x;
	//-------------------------------------------------------
        mGLocalCC<<<aGridDim, aBlockDim, 0, stream>>>
	( gfPadImg, iOffset, iPartSize, m_gfRef, gfPadCC );
}
