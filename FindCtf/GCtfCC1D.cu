#include "CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

#define BLOCK_SIZEX 512

using namespace MotionCor2::FindCtf;

//-----------------------------------------------------------------------------
// 1. The zero-frequency component is at (x=0, y=iCmpY/2). The frequency
//    range in y direction is [-CmpY/2, CmpY/2).
// 2. fFreqLow, fFreqHigh are in the range of [0, 0.5f] of unit 1/pixel.
//-----------------------------------------------------------------------------
static __global__ void mGCalculate
(	float* gfCTF,
	float* gfSpectrum,
	int iSize,
	float fFreqLow,
	float fFreqHigh,
	float fBFactor,
	float* gfRes
)
{	extern __shared__ char s_cArray[];
	float* s_gfCC = (float*)&s_cArray[0];
	float* s_gfStd1 = &s_gfCC[blockDim.x];
	float* s_gfStd2 = &s_gfStd1[blockDim.x];
	s_gfCC[threadIdx.x] = 0.0f;
	s_gfStd1[threadIdx.x] = 0.0f;
	s_gfStd2[threadIdx.x] = 0.0f;
	__syncthreads();
	//--------------
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSize) return;
	//--------------------
	if(x >= fFreqLow && x < fFreqHigh)
	{	float fCTF = x * 0.5f / (iSize - 1);
		float fSpec = gfSpectrum[x]; 
		fCTF = (gfCTF[x] * gfCTF[x] - 0.5f) * expf(-fBFactor * fCTF * fCTF);
		s_gfCC[threadIdx.x] = fCTF * fSpec;
		s_gfStd1[threadIdx.x] = fCTF * fCTF;
		s_gfStd2[threadIdx.x] = fSpec * fSpec;
	}
	__syncthreads();
	//--------------
	x = blockDim.x / 2;
	while(x > 0)
	{	if(threadIdx.x < x)
		{	int j = x + threadIdx.x;
			s_gfCC[threadIdx.x] += s_gfCC[j];
			s_gfStd1[threadIdx.x] += s_gfStd1[j];
			s_gfStd2[threadIdx.x] += s_gfStd2[j];
		}
		__syncthreads();
		x /= 2;
	}
	//-------------
	if(threadIdx.x == 0)
	{	x = 3 * blockIdx.x;
		gfRes[x] = s_gfCC[0];
		gfRes[x+1] = s_gfStd1[0];
		gfRes[x+2] = s_gfStd2[0];
	}
}

GCtfCC1D::GCtfCC1D(void)
{
	m_fBFactor = 1.0f;
	m_gfRes = 0L;
}

GCtfCC1D::~GCtfCC1D(void)
{
	if(m_gfRes != 0L) cudaFree(m_gfRes);
}

void GCtfCC1D::Setup
(	float fFreqLow,  // pixel in Fourier domain
	float fFreqHigh, // pixel in Fourier domain
	float fBFactor
)
{	m_fFreqLow = fFreqLow;
	m_fFreqHigh = fFreqHigh;
	m_fBFactor = fBFactor;
}

void GCtfCC1D::SetSize(int iSize)
{
	if(m_gfRes != 0L) cudaFree(m_gfRes);
	//----------------------------------
	m_iSize = iSize;
	cudaMalloc(&m_gfRes, sizeof(float) * m_iSize);
} 

float GCtfCC1D::DoIt(float* gfCTF, float* gfSpectrum)
{    	
	dim3 aBlockDim(256, 1);
	dim3 aGridDim(1, 1);
	aGridDim.x = (m_iSize + aBlockDim.x - 1) / aBlockDim.x;
	//-----------------------------------------------------
	size_t tBytes = sizeof(float) * aGridDim.x * 3;
	cudaMemset(m_gfRes, 0, tBytes);
	//----------------------------
	tBytes = sizeof(float) * aBlockDim.x * 3;
	mGCalculate<<<aGridDim, aBlockDim, tBytes>>>(gfCTF, gfSpectrum, 
	   m_iSize, m_fFreqLow, m_fFreqHigh, m_fBFactor, m_gfRes);
     	//-----------------------------------------------
	float* pfRes = new float[aGridDim.x * 3];
	tBytes = sizeof(float) * aGridDim.x * 3;
	cudaMemcpy(pfRes, m_gfRes, tBytes, cudaMemcpyDefault);
	//----------------------------------------------------
	double dCC = 0.0, dStd1 = 0.0, dStd2 = 0.0;
	for(int i=0; i<aGridDim.x; i++)
	{	int j = 3 * i;
		dCC += pfRes[j];
		dStd1 += pfRes[j+1];
		dStd2 += pfRes[j+2];
	}
	if(pfRes != 0L) delete[] pfRes;
	//-----------------------------
	if(dStd1 > 0 && dStd2 > 0) dCC /= sqrt(dStd1 * dStd2);
	else dCC = 0.0;
	return (float)dCC;
}

float GCtfCC1D::DoCPU
(	float* gfCTF,
	float* gfSpectrum,
	int iSize
)
{	float* pfCTF = new float[iSize];
	float* pfSpectrum = new float[iSize];
	size_t tBytes = sizeof(float) * iSize;
	cudaMemcpy(pfCTF, gfCTF, tBytes, cudaMemcpyDeviceToHost);
	cudaMemcpy(pfSpectrum, gfSpectrum, tBytes, cudaMemcpyDeviceToHost);
	//-----------------------------------------------------------------
	float fFreqLow = 2 * m_fFreqLow * iSize;
	float fFreqHigh = 2 * m_fFreqHigh * iSize;
	double dXY = 0, dStd1 = 0, dStd2 = 0;
	for(int i=0; i<iSize; i++)
	{	if(i < fFreqLow || i >= fFreqHigh) continue;
		float fX = i * 0.5f / (iSize - 1);
		dXY += (pfCTF[i] * pfSpectrum[i]) * exp(-m_fBFactor * fX * fX);
		dStd1 += (pfCTF[i] * pfCTF[i]);
		dStd2 += (pfSpectrum[i] * pfSpectrum[i]);
	}
	double dCC = dXY / sqrt(dStd1 * dStd2);
	delete[] pfCTF;
	delete[] pfSpectrum;
	return (float)dCC;
}
