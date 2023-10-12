#include "CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

using namespace MotionCor2::FindCtf;

//-----------------------------------------------------------------------------
// 1. The zero-frequency component is at (x=0, y=iCmpY/2). The frequency
//    range in y direction is [-CmpY/2, CmpY/2).
// 2. fFreqLow, fFreqHigh are in the range of [0, 0.5f] of unit 1/pixel.
//-----------------------------------------------------------------------------
static __global__ void mGCalc2D
(	float* gfCTF2D, 
	float* gfSpectrum,
	int iCmpX,
	int iCmpY,
	float fFreqLow2,
	float fFreqHigh2,
	float fBFactor,
	float* gfRes
)
{	extern __shared__ float s_afShared[];
	float* s_afSumStd1 = &s_afShared[blockDim.x];
	float* s_afSumStd2 = &s_afSumStd1[blockDim.x];
	//--------------------------------------------
	float fSumCC = 0.0f, fSumStd1 = 0.0f, fSumStd2 = 0.0f;
	float fX = 0.0f, fY = 0.0f;
	int iOffset = 0, i = 0;
	for(int y=blockIdx.x; y<iCmpY; y+=gridDim.x)
	{	fY = (y - iCmpY * 0.5f) / iCmpY;
		iOffset = y * iCmpX;
		for(int x=threadIdx.x; x<iCmpX; x+=blockDim.x)
		{	fX = (0.5f * x) / (iCmpX - 1);
			fX = fX * fX + fY * fY;
			if(fX <fFreqLow2 || fX > fFreqHigh2) continue;
			//--------------------------------------------
			i = iOffset + x;
			float fC = gfCTF2D[i] * expf(-fBFactor * fX);
			float fS = gfSpectrum[i];
			fSumCC += (fC * fS);
			fSumStd1 += (fC * fC);
			fSumStd2 += (fS * fS);
		}
	}
	s_afShared[threadIdx.x] = fSumCC;
	s_afSumStd1[threadIdx.x] = fSumStd1;
	s_afSumStd2[threadIdx.x] = fSumStd2;
	__syncthreads();
	//----------------------------------		
	iOffset = blockDim.x / 2;
	while(iOffset > 0)
	{	if(threadIdx.x < iOffset)
		{	i = iOffset + threadIdx.x;
			s_afShared[threadIdx.x] += s_afShared[i];
			s_afSumStd1[threadIdx.x] += s_afSumStd1[i];
			s_afSumStd2[threadIdx.x] += s_afSumStd2[i];
		}
		__syncthreads();
		iOffset /= 2;
	}
	//-------------------
	if(threadIdx.x != 0) return;
	i = blockIdx.x * 3;
	gfRes[i] = s_afShared[0];
	gfRes[i+1] = s_afSumStd1[0];
	gfRes[i+2] = s_afSumStd2[0];
}

static __global__ void mGCalc1D(float* gfSum)
{
	extern __shared__ float s_afShared[];
	float* s_afSumStd1 = &s_afShared[blockDim.x];
	float* s_afSumStd2 = &s_afSumStd1[blockDim.x];
	//--------------------------------------------
	int i = threadIdx.x * 3;
	s_afShared[threadIdx.x] = gfSum[i];
	s_afSumStd1[threadIdx.x] = gfSum[i+1];
	s_afSumStd2[threadIdx.x] = gfSum[i+2];
	__syncthreads();
	//------------------------------------
	int iOffset = blockDim.x / 2;
	while(iOffset > 0)
	{	if(threadIdx.x < iOffset)
		{	i = threadIdx.x + iOffset;
			s_afShared[threadIdx.x] += s_afShared[i];
			s_afSumStd1[threadIdx.x] += s_afSumStd1[i];
			s_afSumStd2[threadIdx.x] += s_afSumStd2[i];
		}
		__syncthreads();
		iOffset /= 2;
	}
	//---------------------
	if(threadIdx.x != 0) return;
	float fStd = sqrtf(s_afSumStd1[0] * s_afSumStd2[0]);
	if(fStd == 0) gfSum[0] = 0.0f;
	else gfSum[0] = s_afShared[0] / fStd;
}

GCtfCC2D::GCtfCC2D(void)
{
	m_fBFactor = 1.0f;
	m_gfRes = 0L;
}

GCtfCC2D::~GCtfCC2D(void)
{
	if(m_gfRes != 0L) cudaFree(m_gfRes);
}

void GCtfCC2D::Setup
(	float fFreqLow,  // [0, 0.5]
	float fFreqHigh, // [0, 0.5]
	float fBFactor
)
{	m_fFreqLow = fFreqLow;
	m_fFreqHigh = fFreqHigh;
	m_fBFactor = fBFactor;
}

void GCtfCC2D::SetSize(int* piCmpSize)
{
	if(m_gfRes != 0L) cudaFree(m_gfRes);
	m_aiCmpSize[0] = piCmpSize[0];
	m_aiCmpSize[1] = piCmpSize[1];
	//----------------------------------
	int iSize = m_aiCmpSize[0] * m_aiCmpSize[1];
	double dSize = sqrtf(iSize);
	if(dSize > 512) m_iBlockDimX = 512;
	else if(dSize > 256) m_iBlockDimX = 256;
	else if(dSize > 128) m_iBlockDimX = 128;
	else m_iBlockDimX = 64;
	//---------------------------------------
	m_iGridDimX = (iSize + m_iBlockDimX - 1)/ m_iBlockDimX;
	if(m_iGridDimX > 512) m_iGridDimX = 512;
	else if(m_iGridDimX > 256) m_iGridDimX = 256;
	else if(m_iGridDimX > 128) m_iGridDimX = 128;
	else m_iGridDimX = 64;
	//-------------------------------------------
	cudaMalloc(&m_gfRes, 3 * m_iGridDimX * sizeof(float));
}

float GCtfCC2D::DoIt
(	float* gfCTF, 
	float* gfSpectrum
)
{	dim3 aBlockDim(m_iBlockDimX, 1);
	dim3 aGridDim(m_iGridDimX, 1);
	size_t tSmBytes = sizeof(float) * aBlockDim.x * 3;
	//------------------------------------------------
	float fFreqLow2 = m_fFreqLow / m_aiCmpSize[1];
	float fFreqHigh2 = m_fFreqHigh / m_aiCmpSize[1];
	fFreqLow2 *= fFreqLow2;
	fFreqHigh2 *= fFreqHigh2;
	//-----------------------
	mGCalc2D<<<aGridDim, aBlockDim, tSmBytes>>>(gfCTF, gfSpectrum, 
	   m_aiCmpSize[0], m_aiCmpSize[1], fFreqLow2, fFreqHigh2, 
	   m_fBFactor, m_gfRes);
        //-------------------------------------------------------
	aBlockDim.x = aGridDim.x; aBlockDim.y = 1;
	aGridDim.x = 1; aGridDim.y = 1;
	tSmBytes = sizeof(float) * aBlockDim.x * 3;
	mGCalc1D<<<aGridDim, aBlockDim, tSmBytes>>>(m_gfRes);
	//---------------------------------------------------
	float fCC = 0.0f;
	cudaMemcpy(&fCC, m_gfRes, sizeof(float), cudaMemcpyDefault);
	return fCC;
}

