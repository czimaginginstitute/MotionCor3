#include "CEerUtilInc.h"
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::EerUtil;

static __global__ void mGAddRawFrame
(	unsigned char* gucFrm1,
	unsigned char* gucFrm2,
	unsigned char* gucSum,
	unsigned int uiPixels
)
{	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= uiPixels) return;
	gucSum[i] = gucFrm1[i] + gucFrm2[i];
}

GAddRawFrame::GAddRawFrame(void)
{
}

GAddRawFrame::~GAddRawFrame(void)
{
}

void GAddRawFrame::DoIt
(	unsigned char* gucFrm1,
	unsigned char* gucFrm2,
	unsigned char* gucSum,
	unsigned int uiPixels,
        cudaStream_t stream
)
{	dim3 aBlockDim(512, 1, 1);
	dim3 aGridDim(1, 1, 1);
        aGridDim.x = (uiPixels + aBlockDim.x - 1) / aBlockDim.x;
	mGAddRawFrame<<<aGridDim, aBlockDim, 0, stream>>>
	( gucFrm1, gucFrm2, gucSum, uiPixels
	);
}

