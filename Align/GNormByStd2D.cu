#include "CAlignInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Align;

static __global__ void mGNormByStd2D
(	float* gfImg, 
	int iSizeX,
	int iPadX,
	int iWinX,
	int iWinY
)
{	int ix = blockIdx.x * blockDim.x + threadIdx.x;
	if(ix >= iSizeX) return;
	//----------------------
	int iOffsetX = ix - iWinX / 2;
	int iOffsetY = blockIdx.y - iWinY / 2;
	//------------------------------------------------
	float fMean = 0.0f, fStd = 0.0f, fVal = 0.0f;
	int iCount = 0; 
	for(int j=0; j<iWinY; j++)
	{	int y = j + iOffsetY;
		if(y < 0) continue;
		else if(y >= gridDim.y) continue;
		for(int i=0; i<iWinX; i++)
		{	int x = i + iOffsetX;
			if(x < 0) continue;
			else if(x >= iSizeX) continue;
			fVal = gfImg[y * iPadX + x];
			fMean += fVal;
			fStd += (fVal * fVal);
			iCount += 1;
		}
	}
	fMean /= iCount;
	fStd = fStd / iCount - fMean * fMean;
	//-----------------------------------
	ix = blockIdx.y * iPadX + blockIdx.x * blockDim.x + threadIdx.x;	
	if(fStd < 0) gfImg[ix] = 0.0f;
	else gfImg[ix] /= (fStd + (float)1e-20);
}

GNormByStd2D::GNormByStd2D(void)
{
}

GNormByStd2D::~GNormByStd2D(void)
{
}

void GNormByStd2D::DoIt(float* gfImg, int* piImgSize, bool bPadded, 
	int* piWinSize, cudaStream_t stream)
{
	if(gfImg == 0L) return;
	//---------------------
	int iImageX = bPadded ? (piImgSize[0] / 2 - 1) * 2 : piImgSize[0];
	dim3 aBlockDim(512, 1);
	dim3 aGridDim( (iImageX  + aBlockDim.x - 1)/ aBlockDim.x, 
	   piImgSize[1] );
	mGNormByStd2D<<<aGridDim, aBlockDim, 0, stream>>>(gfImg, iImageX,
	   piImgSize[0], piWinSize[0], piWinSize[1]);
}
