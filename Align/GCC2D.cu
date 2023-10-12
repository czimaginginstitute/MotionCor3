#include "CAlignInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <math.h>

using namespace MotionCor2::Align;

extern __shared__ char s_acArray[];

static __global__ void mGConv
(	cufftComplex* gComp1, 
	cufftComplex* gComp2,
	int iCmpSizeY,
	float fBFactor,
	float* gfCC
)
{	float* sfCC = (float*)&s_acArray[0];
	sfCC[threadIdx.y] = 0.0f;
	__syncthreads();
	//--------------------------------------------
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if((blockIdx.x != 0 || y != 0) && y < iCmpSizeY)
	{	int i = y * gridDim.x + blockIdx.x;
		cufftComplex c1 = gComp1[i];
		cufftComplex c2 = gComp2[i];
		float fX = blockIdx.x / (2.0f * (gridDim.x - 1));
		float fY = y / (float)iCmpSizeY;
		if(fY > 0.5f) fY -= 1.0f;
		sfCC[threadIdx.y] = (c1.x * c2.x + c1.y * c2.y)
	   	   * expf(-fBFactor * (fX * fX + fY * fY))
		   / sqrtf((c1.x * c1.x + c1.y * c1.y)
		   * (c2.x * c2.x + c2.y * c2.y) + (float)1e-30);
	}
	__syncthreads();
	//----------------------------------------------
	y = blockDim.y >> 1;
	while(y > 0)
	{	if(threadIdx.y < y)
		{	sfCC[threadIdx.y] += sfCC[threadIdx.y + y];
		}
		__syncthreads();
		y = y >> 1;
	}
	//----------------------------------------------------------
	if(threadIdx.y != 0) return; 
	gfCC[blockIdx.y * gridDim.x + blockIdx.x] = sfCC[0];
}

static __global__ void mGSum1D
(	float* gfCC,
	int iSize,
	float* gfCCSum
)
{	float* sfCCSum = (float*)&s_acArray[0];
	//-------------------------------------
	int i = 0;
	float sum = 0.0f;
	for(i=threadIdx.x; i<iSize; i+=blockDim.x)
	{	sum += gfCC[i];
	}
	sfCCSum[threadIdx.x] = sum;
	__syncthreads();
	//----------------------------------------
	i = blockDim.x >> 1;
	while(i > 0)
	{	if(threadIdx.x < i)
		{	sfCCSum[threadIdx.x] += sfCCSum[threadIdx.x + i];
		}
		__syncthreads();
		i = i >> 1;
	}
	//---------------------------------------------------------------
	if(threadIdx.x == 0) gfCCSum[0] = sfCCSum[0];
}

GCC2D::GCC2D(void)
{
	m_fBFactor = 10.0f;
	m_gfCC = 0L;
	m_pfCC = 0L;
}

GCC2D::~GCC2D(void)
{
	if(m_gfCC != 0L) cudaFree(m_gfCC);
	if(m_pfCC != 0L) cudaFreeHost(m_pfCC);
}

void GCC2D::SetSize(int* piCmpSize)
{
	m_aiCmpSize[0] = piCmpSize[0];
	m_aiCmpSize[1] = piCmpSize[1];
	//---------------------------------------------
	m_aBlockDim.x = 1;
	m_aBlockDim.y = (m_aiCmpSize[1] / 32) * 32;
	if(m_aBlockDim.y > 1024) m_aBlockDim.y = 1024;
	else if(m_aBlockDim.y < 32) m_aBlockDim.y = 32;
	//---------------------------------------------
	m_aGridDim.x = m_aiCmpSize[0];
	m_aGridDim.y = (m_aiCmpSize[1] + m_aBlockDim.y - 1) /
	   m_aBlockDim.y;
	//---------------------------------------------------
	if(m_gfCC != 0L) cudaFree(m_gfCC);
	int iGridSize = m_aGridDim.x * m_aGridDim.y;
	cudaMalloc(&m_gfCC, sizeof(float) * iGridSize);
	//---------------------------------------------
	if(m_pfCC == 0L) cudaMallocHost(&m_pfCC, sizeof(float));
}

void GCC2D::SetBFactor(float fBFactor)
{
	m_fBFactor = fBFactor;
}

float GCC2D::DoIt
(	cufftComplex* gCmp1, 
	cufftComplex* gCmp2, 
	cudaStream_t stream
)
{	int iShmBytes = sizeof(float) * m_aBlockDim.y;
        mGConv<<<m_aGridDim, m_aBlockDim, iShmBytes, stream>>>(
	   gCmp1, gCmp2, m_aiCmpSize[1], m_fBFactor, m_gfCC);
        //------------------------------------------------------
	int iCCSize = m_aGridDim.x * m_aGridDim.y;
	dim3 aGridDim(1, 1);
	dim3 aBlockDim(1, 1);
	aBlockDim.x = (iCCSize / 32) * 32;
	if(aBlockDim.x > 1024) aBlockDim.x = 1024;
	else if(aBlockDim.x < 32) aBlockDim.x = 32;
	//-----------------------------------------
	iShmBytes = sizeof(float) * aBlockDim.x;
	mGSum1D<<<aGridDim, aBlockDim, iShmBytes, stream>>>(
	   m_gfCC, iCCSize, m_pfCC);
	cudaStreamSynchronize(stream);
	m_pfCC[0] /= (m_aiCmpSize[0] * m_aiCmpSize[1] - 1);
	//----------------------------------------------------
	//mTest(gCmp1, gCmp2);
	return m_pfCC[0];
}

void GCC2D::mTest
(	cufftComplex* gCmp1, 
	cufftComplex* gCmp2 
)
{	int iCmpSize = m_aiCmpSize[0] * m_aiCmpSize[1];
	size_t tBytes = iCmpSize * sizeof(cufftComplex);
	cufftComplex* pCmp1 = new cufftComplex[iCmpSize];
	cufftComplex* pCmp2 = new cufftComplex[iCmpSize];
	cudaMemcpy(pCmp1, gCmp1, tBytes, cudaMemcpyDefault);
	cudaMemcpy(pCmp2, gCmp2, tBytes, cudaMemcpyDefault);
	//--------------------------------------------------
	double dCCSum = 0;
	float fNx = (m_aiCmpSize[0] - 1) * 2.0f;
	for(int i=1; i<iCmpSize; i++)
	{	float fX = (i % m_aiCmpSize[0]) / fNx;
		float fY = (i / m_aiCmpSize[0]) / (float)m_aiCmpSize[1];
		if(fY > 0.5f) fY -= 1.0f;
		//------------------------------------------------------
		fX = (pCmp1[i].x * pCmp2[i].x + pCmp1[i].y * pCmp2[i].y)
		   * (float)exp(-m_fBFactor * (fX * fX + fY * fY));
		dCCSum += fX;
	}
	delete[] pCmp1;
	delete[] pCmp2;
	//-------------
	float fCC = (float)(dCCSum / (iCmpSize - 1));
	printf("GCC2Ds: GPU  CPU: %.5e  %.5e\n", m_pfCC[0], fCC);
		
}
