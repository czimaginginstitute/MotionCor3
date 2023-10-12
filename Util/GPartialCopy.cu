#include "CUtilInc.h"
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2::Util;

template <typename T>
static __global__ void mGPartialCopy
(	T* pSrc, 
	int iSrcX, 
	T* pDst, 
	int iCpyX,
	int iDstX
)
{	int iX = blockIdx.x * blockDim.x + threadIdx.x;
  	int iY = blockIdx.y;
  	if (iX >= iCpyX) return;
  	pDst[iY * iDstX + iX] = pSrc[iY * iSrcX + iX];
}

void GPartialCopy::DoIt
(	float* pSrc,
	int iSrcSizeX,
	float* pDst,
	int iCpySizeX,
	int* piDstSize,
	cudaStream_t stream
)
{	dim3 aBlockDim(64, 1, 1);
        dim3 aGridDim(1, piDstSize[1], 1);
	aGridDim.x = (iCpySizeX + aBlockDim.x - 1) / aBlockDim.x;;
        mGPartialCopy<<<aGridDim, aBlockDim, 0, stream>>>
	( pSrc, iSrcSizeX, pDst, iCpySizeX, piDstSize[0] );
}

void GPartialCopy::DoIt
(	cufftComplex* pSrc,
	int iSrcSizeX,
	cufftComplex* pDst,
	int iCpySizeX,
	int* piDstSize,
	cudaStream_t stream
)
{	dim3 aBlockDim(64, 1, 1);	
	dim3 aGridDim(1, piDstSize[1], 1);
	aGridDim.x = (iCpySizeX + aBlockDim.x - 1 ) / aBlockDim.x;
	mGPartialCopy<<<aGridDim, aBlockDim, 0, stream>>>
	( pSrc, iSrcSizeX, pDst, iCpySizeX, piDstSize[0] );
}

