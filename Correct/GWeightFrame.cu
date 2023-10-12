#include "CCorrectInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2;
using namespace MotionCor2::Correct;

static __device__ __constant__ float gfKvPixSize[2];

//===================================================================
// Weight scheme is based upon Niko lab's formula.
//===================================================================
__device__ float mGCalcWeight(int y, int iCmpY, float fDose)
{	
	float fX = blockIdx.x * 0.5f / (gridDim.x - 1);
	float fY = y / (float)iCmpY;
	if(fY >= 0.5f) fY -= 1.0f;
	fX = sqrtf(fX * fX + fY * fY) / gfKvPixSize[1];
	//---------------------------------------------
	float fCritDose = 0.24499f * powf(fX, -1.6649f) + 2.8141f;
	fCritDose *= gfKvPixSize[0];
	float fWeight = expf(-0.5f * fDose / fCritDose);
	return fWeight; 
}

static __global__ void mGBuildWeightSum2
(	float fDose,
	int iCmpY,
	float* gfWeightSum
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iCmpY) return;
	int i = y * gridDim.x + blockIdx.x;
	if(i == 0) return;
	//---------------------------------
	float fW = mGCalcWeight(y, iCmpY, fDose);
	gfWeightSum[i] += (fW * fW);
}

static __global__ void mGSqrt(int iCmpY, float* gfWeightSum)
{
	int y = blockIdx.y * blockDim.y + threadIdx.y;
        if(y >= iCmpY) return;
        int i = y * gridDim.x + blockIdx.x;
	if(i == 0) return;
	gfWeightSum[i] = sqrtf(gfWeightSum[i]);
}

static __global__ void mGWeight
(	float fDose,
	int iCmpY,
	float* gfWeightSum,
	cufftComplex* gCmpFrame
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
        if(y >= iCmpY) return;
        int i = y * gridDim.x + blockIdx.x;
	if(i == 0) return;
        //----------------
	float fW = mGCalcWeight(y, iCmpY, fDose);
	fW = fW / gfWeightSum[i];
	//-----------------------
	gCmpFrame[i].x *= fW;
	gCmpFrame[i].y *= fW;
}

GWeightFrame::GWeightFrame(void)
{
	m_pfFmDose = 0L;
}

GWeightFrame::~GWeightFrame(void)
{
	this->Clean();
}

void GWeightFrame::Clean(void)
{
	if(m_pfFmDose == 0L) return;
	delete[] m_pfFmDose;
	m_pfFmDose = 0L;
}

void GWeightFrame::BuildWeight
(	float fPixelSize,
	int iKv,
	float* pfFmDose, // accumulated dose
	int* piStkSize,
	float* gfWeightBuf,
	cudaStream_t stream
)
{	this->Clean();
	m_gfWeightSum = gfWeightBuf;
	//--------------------------
	float fKvFactor = 1.0f;
	if(iKv >= 300) fKvFactor = 1.0f;
	else if(iKv < 300 && iKv >= 200) 
	{	fKvFactor = 0.002f * (iKv - 200) + 0.8f;
	}
	else if(iKv < 200 && iKv >= 120) 
	{	fKvFactor = 0.004375f * (iKv - 120) + 0.45f;
	}
	else fKvFactor = 0.45f;
	//---------------------
	m_aiCmpSize[0] = piStkSize[0] / 2 + 1;
	m_aiCmpSize[1] = piStkSize[1];
	m_iNumFrames = piStkSize[2];
	//--------------------------
	if(m_pfFmDose != 0L) delete[] m_pfFmDose;
	m_pfFmDose = new float[m_iNumFrames];
	memcpy(m_pfFmDose, pfFmDose, sizeof(float) * m_iNumFrames);
	//---------------------------------------------------------
	float afKvPixSize[] = {fKvFactor, fPixelSize};
	cudaMemcpyToSymbolAsync(gfKvPixSize, afKvPixSize, sizeof(gfKvPixSize),
	   0, cudaMemcpyDefault, stream);
	//-------------------------------
	int iBytes = sizeof(float) * m_aiCmpSize[0] * m_aiCmpSize[1];
	cudaMemsetAsync(m_gfWeightSum, 0, iBytes, stream);
	//------------------------------------------------
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(m_aiCmpSize[0], 1);
	aGridDim.y = m_aiCmpSize[1] / aBlockDim.y + 1;
	for(int i=0; i<m_iNumFrames; i++)
	{	mGBuildWeightSum2<<<aGridDim, aBlockDim, 0, stream>>>
		   (m_pfFmDose[i], m_aiCmpSize[1], m_gfWeightSum);
	}
	mGSqrt<<<aGridDim, aBlockDim, 0, stream>>>
	( m_aiCmpSize[1], m_gfWeightSum );
}

void GWeightFrame::DoIt
( 	cufftComplex* gCmpFrame,
	int iFrame,
	cudaStream_t stream
)
{	dim3 aBlockDim(1, 128);
	dim3 aGridDim(m_aiCmpSize[0], 1);
	aGridDim.y = (m_aiCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	mGWeight<<<aGridDim, aBlockDim, 0, stream>>>(m_pfFmDose[iFrame], 
	   m_aiCmpSize[1], m_gfWeightSum, gCmpFrame);
}

