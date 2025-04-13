#include "CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

using namespace MotionCor2::FindCtf;

//--------------------------------------------------------------------
// 1. The zero-frequency component is at (x=0, y=iSpectY/2). 
// 2. The frequency range in y direction is [-CmpY/2, CmpY/2).
//--------------------------------------------------------------------
static __global__ void mGCalc2D
(	float* gfCTF2D, 
	float* gfSpect,
	int iSpectX,
	int iSpectY,
	int iWidth,
	float* gfCC
)
{	extern __shared__ float s_afShared[];
	float* s_afMeanC = &s_afShared[blockDim.y];
	float* s_afMeanS = &s_afShared[blockDim.y * 2];
	float* s_afStdC = &s_afShared[blockDim.y * 3];
	float* s_afStdS = &s_afShared[blockDim.y * 4];
	float* s_afCount = &s_afShared[blockDim.y * 5];	
	//-----------------
	int iLow = blockIdx.x - iWidth / 2;
	int iHigh = iLow + iWidth;
	if(iLow < 0)
	{	iLow = 0;
		iHigh = iWidth;
	}
	else if(iHigh > iSpectX) 
	{	iLow = iSpectX - iWidth;
		iHigh = iSpectX; 
	}
	//-----------------
	float fSumCS = 0.0f;
	float fSumMeanC = 0.0f;
	float fSumMeanS = 0.0f;
	float fSumStdC = 0.0f;
	float fSumStdS = 0.0f;
	float fCount = 0.0f;
	//-----------------
	for(int y=threadIdx.y; y<iSpectY; y+=blockDim.y)
	{	float fY = fabsf(y - iSpectY * 0.5f);
		if(fY >= iHigh) continue;
		//----------------
		for(int x=0; x<iSpectX; x++)
		{	if(x >= iHigh) continue;
			//---------------
			float fR = sqrtf(x * x + fY * fY);
			if(fR < iLow || fR >= iHigh) continue;
			//---------------
			int i = y * iSpectX + x;
			float fC = gfCTF2D[i];
			float fS = gfSpect[i];
			fSumMeanC += fC;
			fSumMeanS += fS;
			fSumCS += (fC * fS);
			fSumStdC += (fC * fC);
			fSumStdS += (fS * fS);
			fCount += 1.0f;
		}
	}
	s_afShared[threadIdx.y] = fSumCS;
	s_afMeanC[threadIdx.y] = fSumMeanC;
	s_afMeanS[threadIdx.y] = fSumMeanS;
	s_afStdC[threadIdx.y] = fSumStdC;
	s_afStdS[threadIdx.y] = fSumStdS;
	s_afCount[threadIdx.y] = fCount;
	__syncthreads();
	//-----------------
	int iOffset = blockDim.y / 2;
	while(iOffset > 0)
	{	if(threadIdx.y < iOffset)
		{	int i = iOffset + threadIdx.y;
			s_afShared[threadIdx.y] += s_afShared[i];
			s_afMeanC[threadIdx.y] += s_afMeanC[i];
			s_afMeanS[threadIdx.y] += s_afMeanS[i];
			s_afStdC[threadIdx.y] += s_afStdC[i];
			s_afStdS[threadIdx.y] += s_afStdS[i];
			s_afCount[threadIdx.y] += s_afCount[i];
		}
		__syncthreads();
		iOffset /= 2;
	}
	if(threadIdx.y != 0) return;
	//-----------------
	if(s_afCount[0] == 0)
	{	gfCC[blockIdx.x] = 0.0f;
		return;
	}
	//-----------------
	s_afMeanC[0] /= s_afCount[0];
	s_afMeanS[0] /= s_afCount[0];
	s_afShared[0] /= s_afCount[0];
	s_afStdC[0] /= s_afCount[0];
	s_afStdS[0] /= s_afCount[0];
	//-----------------
	s_afStdC[0] -= (s_afMeanC[0] * s_afMeanC[0]);
	s_afStdS[0] -= (s_afMeanS[0] * s_afMeanS[0]);
	if(s_afStdC[0] <= 0 || s_afStdS[0] <= 0)
	{	gfCC[blockIdx.x] = 0.0f;
		return;
	}
	//-----------------
	s_afShared[0] -= (s_afMeanC[0] * s_afMeanS[0]);
	gfCC[blockIdx.x] = s_afShared[0] / sqrtf(s_afStdC[0] * s_afStdS[0]);
}

GSpectralCC2D::GSpectralCC2D(void)
{
	m_gfCC = 0L;
	m_pfCC = 0L;
	m_aiSpectSize[0] = 0;
	m_aiSpectSize[1] = 0;
}

GSpectralCC2D::~GSpectralCC2D(void)
{
	if(m_gfCC != 0L) cudaFree(m_gfCC);
	if(m_pfCC != 0L) delete[] m_pfCC;
}

//--------------------------------------------------------------------
// 1. piSpectSize is the size of half spectrum.
//--------------------------------------------------------------------
void GSpectralCC2D::SetSize(int* piSpectSize)
{	
	int iOldSize = m_aiSpectSize[0] * m_aiSpectSize[1];
	int iNewSize = piSpectSize[0] * piSpectSize[1];
	//-----------------
	m_aiSpectSize[0] = piSpectSize[0];
	m_aiSpectSize[1] = piSpectSize[1];
	if(iOldSize > iNewSize) return;
	//-----------------
	if(m_gfCC != 0L) cudaFree(m_gfCC);
	if(m_pfCC != 0L) delete[] m_pfCC;
	cudaMalloc(&m_gfCC, m_aiSpectSize[0] * sizeof(float));
	m_pfCC = new float[m_aiSpectSize[0]];
}

int GSpectralCC2D::DoIt
(	float* gfCTF, 
	float* gfSpect
)
{	dim3 aBlockDim(1, 512);
	dim3 aGridDim(m_aiSpectSize[0], 1);
	size_t tSmBytes = sizeof(float) * aBlockDim.y * 6;
	//-----------------
	mGCalc2D<<<aGridDim, aBlockDim, tSmBytes>>>(gfCTF, gfSpect, 
	   m_aiSpectSize[0], m_aiSpectSize[1], 10, m_gfCC);
	cudaMemcpy(m_pfCC, m_gfCC, m_aiSpectSize[0] * sizeof(float),
	   cudaMemcpyDefault);
        //-----------------
	int iMax = 1;
	float fMax = (float)-1e30;
	for(int i=1; i<m_aiSpectSize[0]; i++)
	{	if(m_pfCC[i] <= fMax) continue;
		fMax = m_pfCC[i];
		iMax = i;
	}
	if(fMax < 0.143f) return iMax;
	//-----------------
	int iShell = iMax;
	for(int i=iMax; i<m_aiSpectSize[0]; i++)
	{	if(m_pfCC[i] < 0.143f) break;
		iShell = i;
	}
	return iShell;
}

