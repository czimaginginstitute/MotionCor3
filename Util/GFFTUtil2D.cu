#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGMultiplyFactor
(	cufftComplex* gCmp,
	int nxy,
	float fFactor
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= nxy) return;
	gCmp[i].x *= fFactor;
	gCmp[i].y *= fFactor;
}

static __global__ void mGGetAmp
(	cufftComplex* gCmp,
	int nxy,
	float* gfAmp
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= nxy) return;
	gfAmp[i] = sqrtf(gCmp[i].x * gCmp[i].x + gCmp[i].y * gCmp[i].y);
}

static __global__ void mGShiftFrame
(       cufftComplex* gComp,
        float fShiftX,
        float fShiftY,
        int iCmpX,
        int iCmpY
)
{       int x = blockIdx.x * blockDim.x + threadIdx.x;
        int y = blockIdx.y * blockDim.y + threadIdx.y;
        if (x >= iCmpX || y >= iCmpY) return;
        int i = y * iCmpX + x;
	//---------------------------------
	if(y  > (iCmpY / 2)) y -= iCmpY;
        float fPhaseShift = x * fShiftX + y * fShiftY;
        float fCos = cosf(fPhaseShift);
        float fSin = sinf(fPhaseShift);
        //-----------------------------
	float fRe = fCos * gComp[i].x - fSin * gComp[i].y;
        float fIm = fCos * gComp[i].y + fSin * gComp[i].x;
        gComp[i].x = fRe;
        gComp[i].y = fIm;
}

GFFTUtil2D::GFFTUtil2D(void)
{
}

GFFTUtil2D::~GFFTUtil2D(void)
{
}

void GFFTUtil2D::Multiply
( 	cufftComplex* gComp,
	int* piCmpSize,
	float fFactor,
        cudaStream_t stream
)
{	int nxy = piCmpSize[0] * piCmpSize[1];
        dim3 aBlockDim(512, 1, 1);
        dim3 aGridDim((nxy + aBlockDim.x - 1) / aBlockDim.x, 1, 1);
	//----------------------------------
	mGMultiplyFactor<<<aGridDim, aBlockDim, 0, stream>>> (gComp, nxy, fFactor);
}

void GFFTUtil2D::GetAmp
(	cufftComplex* gComp,
	int* piCmpSize,
	float* pfAmpRes,
	bool bGpuRes,
        cudaStream_t stream
)
{	int nxy = piCmpSize[0] * piCmpSize[1];
        dim3 aBlockDim(512, 1, 1);
        dim3 aGridDim((nxy + aBlockDim.x - 1) / aBlockDim.x, 1, 1);
	//------------------------------
	size_t tBytes = sizeof(float) * nxy;
	float* gfAmp = nullptr;
	if(bGpuRes) gfAmp = pfAmpRes;
	else cudaMalloc(&gfAmp, tBytes);
	//----------------------------------
	mGGetAmp<<<aGridDim, aBlockDim, 0, stream>>> (gComp, nxy, gfAmp);
	//---------------------------
	if(bGpuRes) return;
	cudaMemcpy(pfAmpRes, gfAmp, tBytes, cudaMemcpyDeviceToHost);
	cudaFree(gfAmp);
}
	
void GFFTUtil2D::Shift
(       cufftComplex* gComp,
	int* piCmpSize,
        float* pfShift,
        cudaStream_t stream
)
{       if(pfShift == 0L) return;
        if(pfShift[0] == 0.0f && pfShift[1] == 0.0f) return;
        //--------------------------------------------------
	dim3 aBlockDim(128, 4, 1);
        int iGridX = (piCmpSize[0] + aBlockDim.x - 1) / aBlockDim.x;
        int iGridY = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
        dim3 aGridDim(iGridX, iGridY);
        //----------------------------------
	int iNx = 2 * (piCmpSize[0] - 1);
        float f2PI = (float)(8 * atan(1.0));
        float fShiftX = pfShift[0] * f2PI / iNx;
        float fShiftY = pfShift[1] * f2PI / piCmpSize[1];
        //-----------------------------------------------
	mGShiftFrame<<<aGridDim, aBlockDim, 0, stream>>>
        (  gComp, fShiftX, fShiftY, piCmpSize[0], piCmpSize[1]
        );
}
