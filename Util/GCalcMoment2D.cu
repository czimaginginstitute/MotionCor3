#include "CUtilInc.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGSum2D
(	float* gfImg,
        int iSizeX,
	int iSizeY,
        int iPadX,
	int iExp,
	float* gfSum
)
{	extern __shared__ float shared[];
	float sum = 0.0f;
	for (int y=blockIdx.x; y<iSizeY; y+=gridDim.x) 
	{	float *ptr = gfImg + y * iPadX;
		for (int ix=threadIdx.x; ix<iSizeX; ix+=blockDim.x) 
		{	float val = ptr[ix];
			float expval = val;
			for(int i=1; i<iExp; i++)
			{	expval *= val;
			}
            		sum += expval;
          	}
        }
	//----------------------------
	shared[threadIdx.x] = sum;
	__syncthreads();
	//--------------
	for (int offset=blockDim.x/2; offset>0; offset=offset/2) 
	{	if (threadIdx.x < offset)
		{	shared[threadIdx.x] += shared[threadIdx.x+offset];
		}
		__syncthreads();
	}
        if (threadIdx.x != 0) return;
	else gfSum[blockIdx.x] = shared[0] / (iSizeX * iSizeY);
}

static __global__ void mGSum1D (float* gfSum)
{
        extern __shared__ float shared[];
	shared[threadIdx.x] = gfSum[threadIdx.x];
	__syncthreads();
	//--------------
	for (int offset=blockDim.x/2; offset>0; offset=offset/2) 
	{	if (threadIdx.x < offset)
		{	shared[threadIdx.x] += shared[threadIdx.x+offset];
		}
		__syncthreads();
	}
        if (threadIdx.x == 0) gfSum[0] = shared[0];
}

GCalcMoment2D::GCalcMoment2D(void)
{
	m_aBlockDim.x = 512;
	m_aBlockDim.y = 1;
	m_aGridDim.x = 512;
	m_aGridDim.y = 1;
	m_gfBuf = 0L;
}

GCalcMoment2D::~GCalcMoment2D(void)
{
	this->Clean();
}

void GCalcMoment2D::Clean(void)
{
	if(m_gfBuf == 0L) return;
	cudaFree(m_gfBuf);
	m_gfBuf = 0L;
}

void GCalcMoment2D::SetSize(int* piImgSize, bool bPadded)
{
	this->Clean();
	m_iPadX = piImgSize[0];
	m_aiImgSize[1] = piImgSize[1];
	m_aiImgSize[0] = bPadded ? (m_iPadX/2 - 1) * 2 : m_iPadX;
	//-------------------------------------------------------
	int iPixels = m_aiImgSize[0] * m_aiImgSize[1];
	int iNumBlocks = (iPixels + m_aBlockDim.x - 1) / m_aBlockDim.x;
	if(iNumBlocks >= 512) m_aGridDim.x = 512;
	else if(iNumBlocks >= 128) m_aGridDim.x = 128;
	else if(iNumBlocks >= 64) m_aGridDim.x = 64;
	else m_aGridDim.x = 32;
	//---------------------
	cudaMalloc(&m_gfBuf, sizeof(float) * m_aGridDim.x);
}

float GCalcMoment2D::DoIt
(	float* gfImg, 
	int iExponent,
	bool bSync,
	cudaStream_t stream
)
{	int iShmBytes = sizeof(float) * m_aBlockDim.x;
	mGSum2D<<<m_aGridDim, m_aBlockDim, iShmBytes, stream>>>(gfImg, 
	   m_aiImgSize[0], m_aiImgSize[1], m_iPadX, iExponent, m_gfBuf);
        mGSum1D<<<1, m_aGridDim, iShmBytes, stream>>>(m_gfBuf);
	//----------------------------------------------------
	if(bSync || stream == 0)
	{	float fMoment = this->GetResult();
		return fMoment;
	}
	else return 0.0f;
}

float GCalcMoment2D::GetResult(void)
{
	float fRes = 0.0f;
	cudaMemcpy(&fRes, m_gfBuf, sizeof(float), cudaMemcpyDefault);
	return fRes;
}

void GCalcMoment2D::Test(float* gfImg, float fExp)
{
	int iPixels = m_iPadX * m_aiImgSize[1];
	float* pfImg = new float[iPixels];
	cudaMemcpy(pfImg, gfImg, iPixels * sizeof(float), cudaMemcpyDefault);
	double dMoment = 0;
	for(int y=0; y<m_aiImgSize[1]; y++)
	{	int i = y * m_iPadX;
		for(int x=0; x<m_aiImgSize[0]; x++)
		{	dMoment += pow(pfImg[i+x], fExp);
		}
	}
	dMoment = dMoment / (m_aiImgSize[0] * m_aiImgSize[1]);
	printf("GCalcMoment2D CPU Res: %.3e\n", dMoment);
	delete[] pfImg;
}

