#include "CAlignInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace DfCorr::Align;

static __global__ void mGNormByStd2D
(	float* gfImg, 
	int iSizeX,
	int iPadX,
	int iWinX,
	int iWinY
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	//---------------------
	int iStartX = x - iWinX / 2;
	if(iStartX < 0) iStartX = 0;
	else if((iStartX + iWinX) > iSizeX) iStartX = iSizeX - iWinX;
	int iStartY = (int)blockIdx.y - iWinY / 2;
	if(iStartY < 0) iStartY = 0;
	else if((iStartY + iWinY) > gridDim.y) iStartY = gridDim.y - iWinY;
	//-----------------------------------------------------------------
	float* gfBlock = gfImg + iStartY * iPadX + iStartX;
	float fMean = 0.0f, fStd = 0.0f, fVal = 0.0f;
	float fFact = 1.0f / (iWinX * iWinX); 
	for(int y=0; y<iWinY; y++)
	{	for(x=0; x<iWinX; x++)
		{	fVal = gfBlock[y * iPadX + x];
			fMean += (fVal * fFact);
			fStd += (fVal * fVal * fFact);
		}
	}
	fStd = fStd - fMean * fMean;
	//--------------------------
	x = blockIdx.y * iPadX + blockIdx.x * blockDim.x + threadIdx.x;	
	if(fStd < 0) gfImg[x] = 0.0f;
	else gfImg[x] /= (fStd + (float)1e-20);
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
