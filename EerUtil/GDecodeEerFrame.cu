#include "CEerUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::EerUtil;

//-------------------------------------------------------------------
// 1. The gain-corrected frame could be padded for FFT. In this
//    case iPadX = (iSizeX / 2 + 1) * 2. Otherwise iPadX = iSizeX.
//-------------------------------------------------------------------
static __global__ void mGDecode7Bits
(	unsigned char* gucEerFrame,
	unsigned char* gucZeros,
	unsigned char* gucSubpixs,
	int iNumChunks
)
{	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= iNumChunks) return;
	//-------------------------
	unsigned int iV = i * 11;
	unsigned int iBitOffset = iV % 8;
	iV = *((unsigned int*)(gucEerFrame + (iV >> 3)));
	iV = iV >> iBitOffset;
	//--------------------
	gucZeros[i] = (unsigned char)(iV & 127);
	gucSubpixs[i] = (unsigned char)(((iV >> 7) & 15) ^ 0x0A); 
}

static __global__ void mGDecode8Bits
(	unsigned char* gucEerFrame,
	unsigned char* gucZeros,
	unsigned char* gucSubpixs,
	int iNumChunks
)
{	unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= iNumChunks) return;
	//-------------------------
	unsigned int iV = i * 12;
	unsigned int iBitOffset = iV % 8;
	iV = *((unsigned int*)(gucEerFrame + (iV >> 3)));
	iV = iV >> iBitOffset;
	//--------------------
	gucZeros[i] = (unsigned char)(iV & 255);
	gucSubpixs[i] = (unsigned char)(((iV >> 8) & 15) ^ 0x0A);
}

GDecodeEerFrame::GDecodeEerFrame(void)
{
}

GDecodeEerFrame::~GDecodeEerFrame(void)
{
}

void GDecodeEerFrame::DoIt
(	unsigned char* gucEerFrame,
	unsigned char* gucZeros,
	unsigned char* gucSubpixs,
	int iEerFrameBytes,
	bool b7Bits,
	cudaStream_t stream
)
{	unsigned int iChunkSize = b7Bits ? 11 : 12;
	unsigned int iNumChunks = iEerFrameBytes / iChunkSize;
	//----------------------------------------------------
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(1, 1);
	aGridDim.x = (iNumChunks + aBlockDim.x - 1) / aBlockDim.x;
	//--------------------------------------------------------
	if(b7Bits)
	{	mGDecode7Bits<<<aGridDim, aBlockDim, 0, stream>>>
		( gucEerFrame, gucZeros, gucSubpixs, iNumChunks
		);
	}
	else
	{	mGDecode8Bits<<<aGridDim, aBlockDim, 0, stream>>>
		( gucEerFrame, gucZeros, gucSubpixs, iNumChunks
		);
	}
}
