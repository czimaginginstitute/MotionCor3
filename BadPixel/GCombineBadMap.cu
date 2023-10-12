#include "CDetectMain.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::BadPixel;

static __global__ void mGCombine
(	unsigned char* gucMap1,
	unsigned char* gucMap2,
	int iSizeX,
	unsigned char* gucRes
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	//---------------------
	int i = blockIdx.y * iSizeX + x;
	gucRes[i] = gucMap1[i] | gucMap2[i];
}

GCombineMap::GCombineMap(void)
{
}

GCombineMap::~GCombineMap(void)
{
}

void GCombineMap::GDoIt
(	unsigned char* gucMap1,
	unsigned char* gucMap2,
	unsigned char* gucResMap,
	int* piMapSize
)
{	dim3 aBlockDim(1024, 1), aGridDim(1, piMapSize[1]);
	aGridDim.x = piMapSize[0] / aBlockDim.x;
	if((piMapSize[0] % aBlockDim.x) > 0) aGridDim.x += 1;
	//---------------------------------------------------
	mGCombine<<<aGridDim, aBlockDim>>>(gucMap1, gucMap2,
	   piMapSize[0], gucResMap);
}

unsigned char* GCombineMap::GCopyMap
(	unsigned char* pucMap,
	int* piMapSize
)
{	unsigned char* gucBuf = 0L;
	size_t tBytes = piMapSize[0] * piMapSize[1] * sizeof(char);
	cudaMalloc(&gucBuf, tBytes);
	cudaMemcpy(gucBuf, pucMap, tBytes, cudaMemcpyDefault);
	return gucBuf;
}
