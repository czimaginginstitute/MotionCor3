#include "CFindCtfInc.h"
#include "../Util/CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2;
using namespace MotionCor2::FindCtf;

//-----------------------------------------------------------------------------
// 1. Calculate the logrithmic amplitude spectrum given gComp, the Fourier
//    transform.
// 2. The spectrum is the half spectrum with x frequency ranging from 0 to
//    0.5 and y frequency form -0.5 to +0.5.
// 3. The zero frequency is at (0, iCmpY / 2).
//-----------------------------------------------------------------------------
static __global__ void mGCalculate
(	cufftComplex* gComp, 
	float* gfSpectrum,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iCmpY) return;
	int i = y * gridDim.x + blockIdx.x;
	//--------------------------------------------
	// put DC at x = 0, y = iCmpY / 2
	//--------------------------------------------
	y = y + iCmpY / 2;
	if(y >= iCmpY) y = y - iCmpY;
	//---------------------------
	float fAmp2 = gComp[i].x * gComp[i].x + gComp[i].y * gComp[i].y;
	gfSpectrum[y * gridDim.x + blockIdx.x] = sqrtf(fAmp2); 
}

static __global__ void mGLogrithm
(	float* gfSpectrum,
	int iSizeY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iSizeY) return;
	int i = y * gridDim.x + blockIdx.x;
	gfSpectrum[i] = logf(gfSpectrum[i]);
}

//------------------------------------------------------------------------------
// 1. DC of gfHalfSpect is already at (0, iCmpY / 2)
// 2. DC of gfFullSpect will be at (iCmpX / 2, iCmpY / 2)
//------------------------------------------------------------------------------
static __global__ void mGenFullSpect
(	float* gfHalfSpect,
	float* gfFullSpect,
	int iHalfX, int iCmpY,
	int iFullSizeX
)
{	int xSrc, ySrc, xDst, yDst;
	yDst = blockIdx.y * blockDim.y + threadIdx.y;
	if(yDst >= iCmpY) return;
	//-----------------------
	xDst = blockIdx.x - iHalfX;
	if(xDst >= 0)
	{	xSrc = xDst;
		ySrc = yDst;
	}
	else
	{	xSrc = -xDst;
		ySrc = iCmpY - yDst;
	}
	gfFullSpect[yDst * iFullSizeX + blockIdx.x] = 
	   gfHalfSpect[ySrc * (iHalfX + 1) + xSrc];
}

GCalcSpectrum::GCalcSpectrum(void)
{
}

GCalcSpectrum::~GCalcSpectrum(void)
{
}

void GCalcSpectrum::DoIt
(	cufftComplex* gCmp, 
	float* gfSpectrum, 
	int* piCmpSize,
	bool bLog
)
{	dim3 aBlockDim(1, 512);
	int iGridY = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	dim3 aGridDim(piCmpSize[0], iGridY);
	mGCalculate<<<aGridDim, aBlockDim>>>(gCmp, gfSpectrum, piCmpSize[1]);
	if(!bLog) return;
	//---------------
	mGLogrithm<<<aGridDim, aBlockDim>>>(gfSpectrum, piCmpSize[1]);
}

void GCalcSpectrum::DoPad
(	float* gfPadImg,
	float* gfSpectrum,
	int* piPadSize,
	bool bLog
)
{	Util::CCufft2D cufft2D;
	bool bPad = true, bNorm = true;
	cufft2D.CreateForwardPlan(piPadSize, bPad);
	cufft2D.Forward(gfPadImg, !bNorm);
	cudaStreamSynchronize((cudaStream_t)0);
	cufft2D.DestroyPlan();
	//-----------------
	int aiCmpSize[] = {piPadSize[0]/2, piPadSize[1]};
	this->DoIt((cufftComplex*)gfPadImg, gfSpectrum, aiCmpSize, bLog);
}

void GCalcSpectrum::Logrithm
(	float* gfSpectrum,
	int* piSize
)
{	dim3 aBlockDim(1, 512);
	dim3 aGridDim(piSize[0], piSize[1]/aBlockDim.y+1);
	mGLogrithm<<<aGridDim, aBlockDim>>>(gfSpectrum, piSize[1]);
}

void GCalcSpectrum::GenFullSpect
(	float* gfHalfSpect, 
	int* piCmpSize,
	float* gfFullSpect,
	bool bFullPadded
)
{	int iHalfX = piCmpSize[0] - 1;
	int iNx = iHalfX * 2;
	int iFullSizeX = bFullPadded ? iNx + 2 : iNx;
	//-------------------
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(iNx, 1);
	aGridDim.y = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	//----------------------------------------------------------
	mGenFullSpect<<<aGridDim, aBlockDim>>>(gfHalfSpect,
	   gfFullSpect, iHalfX, piCmpSize[1], iFullSizeX);
}	
