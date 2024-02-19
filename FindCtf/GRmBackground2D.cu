#include "CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::FindCtf;

//-----------------------------------------------------------------------------
// 1. gfIntSpect is shifted with its DC at (0, iCmpY/2).
// 2. See GCalcSpectrum.cu kernel mGCalculate.
//-----------------------------------------------------------------------------
static __global__ void mGRemove
(	float* gfInSpect,
	float* gfOutSpect,
	int iCmpY,
	int iBoxSize
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iCmpY) return;
	int i = y * gridDim.x + blockIdx.x;
	//---------------------------------
	int iHalfX = gridDim.x - 1;
	int iHalfY = iCmpY / 2;
	//-------------------------
	y -= iHalfY;
	float fR = blockIdx.x * 0.5f / (gridDim.x - 1);
	fR = sqrtf(fR * fR + y * y / (float)(iCmpY * iCmpY));
	if(fR < 0.04)
	{	gfOutSpect[i] = 0.0f;
		return;
	}
	//--------------------------------------------------------
	// (iX, iY): origin at lower left corner
	// (xxm yy): origin at iX = 0, iY = iHalfY
	// (xx = 0, yy=0) and (iX = 0, iY=iHalfY) is DC component
	//--------------------------------------------------------
	int iX = 0, iY = 0;
	int iHalfBox = iBoxSize / 2;
	float fBackground = 0.0f;
	for(int k=-iHalfBox; k<=iHalfBox; k++)
	{	int yy = k + y;
		for(int j=-iHalfBox; j<=iHalfBox; j++)
		{	int xx = j + blockIdx.x;
			if(xx >= iHalfX) xx = xx - 2 * iHalfX;
			if(xx >= 0)
			{	iX = xx;
				iY = iHalfY + yy;
			}
			else
			{	iX = -xx;
				iY = iHalfY - yy;
			}
			if(iY < 0) iY += iCmpY;
			else if(iY >= iCmpY) iY -= iCmpY;
			//-------------------------------
			iX = iY * gridDim.x + iX;
			fBackground += sqrtf(gfInSpect[iX]);
          	}
	}
	fBackground /= (iBoxSize * iBoxSize);
	fBackground = (fBackground * fBackground);
	gfOutSpect[i] = gfInSpect[i] - fBackground;
}

GRmBackground2D::GRmBackground2D(void)
{
}

GRmBackground2D::~GRmBackground2D(void)
{
}

//------------------------------------------------------------------------------
// 1. fBoxSize defines a square box where the mean is calculated. The mean is
//    then subtracted from the central pixel in the box.
// 2. This operation is performed at each pixel.
//------------------------------------------------------------------------------
void GRmBackground2D::DoIt
(	float* gfInSpect,
	float* gfOutSpect,	
	int* piCmpSize,
	int iBoxSize
)
{	dim3 aBlockDim(1, 512);
	dim3 aGridDim(piCmpSize[0], 1);
	aGridDim.y = piCmpSize[1] / aBlockDim.y;
	if((piCmpSize[1] % aBlockDim.y) > 0) aGridDim.y++;
	//-----------------
	mGRemove<<<aGridDim, aBlockDim>>>(gfInSpect, gfOutSpect,
	   piCmpSize[1], iBoxSize);
}

