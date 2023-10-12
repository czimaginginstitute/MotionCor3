#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGResize
( 	cufftComplex* gCmpIn,
	int iCmpSizeInX,
	int iCmpSizeInY,
  	cufftComplex* gCmpOut, 
	int iCmpSizeOutY,
	bool bSum
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iCmpSizeOutY) return;
	//---------------------------
	int iOut = y * gridDim.x + blockIdx.x;
	if(blockIdx.x >= iCmpSizeInX) return;
	//-----------------------------------
        if(y > (iCmpSizeOutY / 2)) 
	{	y -= iCmpSizeOutY;
		if(y <= (-iCmpSizeInY / 2)) return;
		else y += iCmpSizeInY;
	}
	else
	{	if(y > (iCmpSizeInY / 2)) return;
	}
	//---------------------------------------
	int iIn = y * iCmpSizeInX + blockIdx.x;
	if(bSum)
	{	gCmpOut[iOut].x += gCmpIn[iIn].x;
		gCmpOut[iOut].y += gCmpIn[iIn].y;
	}
	else
	{	gCmpOut[iOut].x = gCmpIn[iIn].x;
		gCmpOut[iOut].y = gCmpIn[iIn].y;
	}
}

GFourierResize2D::GFourierResize2D(void)
{
}

GFourierResize2D::~GFourierResize2D(void)
{
}

void GFourierResize2D::GetBinnedCmpSize
(	int* piCmpSize,
	float fBin,
	int* piNewSize // cmp size after binning
)
{	piNewSize[0] = piCmpSize[0];
	piNewSize[1] = piCmpSize[1];
	if(fBin == 1) return;
	//-------------------
	int aiImgSize[2] = {0};
	aiImgSize[0] = (piCmpSize[0] - 1) * 2;
	aiImgSize[1] = piCmpSize[1];
	GFourierResize2D::GetBinnedImgSize(aiImgSize, fBin, piNewSize);
	piNewSize[0] = piNewSize[0] / 2 + 1;
	piNewSize[1] = piNewSize[1]; 
}

void GFourierResize2D::GetBinnedImgSize
(	int* piImgSize,
	float fBin,
	int* piNewSize
)
{	piNewSize[0] = piImgSize[0];
	piNewSize[1] = piImgSize[1];
	if(fBin == 1.0f) return;
	//--------------------------
	piNewSize[0] = (int)(piImgSize[0] / fBin);
	piNewSize[1] = (int)(piImgSize[1] / fBin);
	//----------------------------------------
	piNewSize[0] = piNewSize[0] / 2 * 2;
	piNewSize[1] = piNewSize[1] / 2 * 2;
}

float GFourierResize2D::CalcPixSize
(	int* piImgSize,
	float fBin,
	float fPixSize  // before binning
)
{	int aiNewSize[2] = {0};
	GFourierResize2D::GetBinnedImgSize(piImgSize, fBin, aiNewSize);
	float fPixSizeX = piImgSize[0] * fPixSize / aiNewSize[0];
	float fPixSizeY = piImgSize[1] * fPixSize / aiNewSize[1];
	return (fPixSizeX + fPixSizeY) * 0.5f;
}

void GFourierResize2D::GetBinning
(	int* piCmpSize, 
	int* piNewSize,
	float* pfBinning
)
{	pfBinning[0] = 1.0f;
	pfBinning[1] = 1.0f;
	//------------------
	if(piCmpSize[0] != piNewSize[0])
	{	pfBinning[0] = (piCmpSize[0] - 1.0f) / (piCmpSize[0] - 1.0f);
	}
	if(piCmpSize[1] != piNewSize[1])
	{	pfBinning[1] = piCmpSize[1] / (float)piCmpSize[1];
	}
}

void GFourierResize2D::DoIt
( 	cufftComplex* gCmpIn, 
	int* piSizeIn,
  	cufftComplex* gCmpOut, 
	int* piSizeOut,
	bool bSum,
	cudaStream_t stream
)
{	dim3 aBlockDim(1, 64);
	dim3 aGridDim(piSizeOut[0], 1);
	aGridDim.y = (piSizeOut[1] + aBlockDim.y - 1) / aBlockDim.y;
	//----------------------------------------------------------
	mGResize<<<aGridDim, aBlockDim, 0, stream>>>
	( gCmpIn, piSizeIn[0], piSizeIn[1], 
	  gCmpOut, piSizeOut[1], bSum );	
}

