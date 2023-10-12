#include "CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

static __global__ void mGPack
(	unsigned char* pucRawFrm,
	unsigned char* pucPkdFrm,
	int iSizeX,
	int iSizeY,
	int iPackX
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
        if(y >= iSizeY) return;
	//---------------------
	int x2 = blockIdx.x * 2;
        int i = y * iPackX + blockIdx.x;
	int j = y * iSizeX + x2;
	//----------------------
	pucPkdFrm[i] = pucRawFrm[j] & 0xf;
	if((x2+1) >= iSizeX) return;
	pucPkdFrm[i] += ((pucRawFrm[j+1] & 0xf) << 4); 
}

static __global__ void mGUnpack
(	unsigned char* gucPkdFrm,
	unsigned char* gucRawFrm,
	int iPackX,
	int iSizeX
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	int i = blockIdx.y * iSizeX + x;
	int j = blockIdx.y * iPackX + x / 2;
	if((x % 2) == 0) gucRawFrm[i] = gucPkdFrm[j] & 0xf;
	else gucRawFrm[i] = (gucPkdFrm[j] >> 4) & 0xf;
}

G4BitImage::G4BitImage(void)
{
}

G4BitImage::~G4BitImage(void)
{
}

int G4BitImage::GetPkdSize(int iSize)
{
	return (iSize + 1) / 2;
}

void G4BitImage::GetPkdImgSize(int* piRawSize, int* piPkdSize)
{
	piPkdSize[0] = G4BitImage::GetPkdSize(piRawSize[0]);
	piPkdSize[1] = piRawSize[1];
}

void G4BitImage::Pack
(	unsigned char* gucRawFrm,
	unsigned char* gucPkdFrm,
	int* piRawSize
)
{	int iPackX = G4BitImage::GetPkdSize(piRawSize[0]);
	dim3 aBlockDim(1, 128);
	dim3 aGridDim(iPackX, 1);
	aGridDim.y = (piRawSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	mGPack<<<aGridDim, aBlockDim>>>(gucRawFrm, gucPkdFrm, 
	   piRawSize[0], piRawSize[1], iPackX);
}

void G4BitImage::Unpack
(	unsigned char* gucPkdFrm,
	unsigned char* gucRawFrm,
	int* piRawSize
)
{	dim3 aBlockDim(128, 1);
	dim3 aGridDim(1, piRawSize[1]);
	aGridDim.x = (piRawSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	int iPackX = G4BitImage::GetPkdSize(piRawSize[0]);
	mGUnpack<<<aGridDim, aBlockDim>>>(gucPkdFrm, gucRawFrm, 
	   iPackX, piRawSize[0]);
}

