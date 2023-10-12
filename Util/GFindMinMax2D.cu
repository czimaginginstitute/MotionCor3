#include "CUtilInc.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::Util;

static __global__ void mGFindMin2D
(	float* gfImg,
        int iSizeX,
	int iSizeY,
        int iPadX,
	float* gfMin
)
{	extern __shared__ float shared[];
	float fMin = (float)1e20;
	for (int y=blockIdx.x; y<iSizeY; y+=gridDim.x) 
	{	float *ptr = gfImg + y * iPadX;
		for (int x=threadIdx.x; x<iSizeX; x+=blockDim.x) 
		{	fMin = fminf(fMin, ptr[x]);
          	}
        }
	//-----------------------------------------
	shared[threadIdx.x] = fMin;
	__syncthreads();
	//--------------
	for (int offset=blockDim.x/2; offset>0; offset=offset/2) 
	{	if (threadIdx.x < offset)
		{	shared[threadIdx.x] = fminf(shared[threadIdx.x],
			   shared[threadIdx.x + offset]);
		}
		__syncthreads();
	}
        if (threadIdx.x == 0) gfMin[blockIdx.x] = shared[0];
}

static __global__ void mGFindMax2D
(       float* gfImg,
        int iSizeX,
        int iSizeY,
        int iPadX,
        float* gfMax
)
{	extern __shared__ float shared[];
	float fMax = (float)-1e20;
	for (int y=blockIdx.x; y<iSizeY; y+=gridDim.x)
	{	float *ptr = gfImg + y * iPadX;
		for (int x=threadIdx.x; x<iSizeX; x+=blockDim.x)
		{	fMax = fmaxf(fMax, ptr[x]);
		}
	}
	//-----------------------------------------
	shared[threadIdx.x] = fMax;
	__syncthreads();
	//--------------
	for (int offset=blockDim.x/2; offset>0; offset=offset/2)
	{	if (threadIdx.x < offset)
		{	shared[threadIdx.x] = fmaxf(shared[threadIdx.x],
			   shared[threadIdx.x + offset]);
		}
		__syncthreads();
	}
	if (threadIdx.x == 0) gfMax[blockIdx.x] = shared[0];
}

static __global__ void mGFindMin1D (float* gfMin)
{
        extern __shared__ float shared[];
	shared[threadIdx.x] = gfMin[threadIdx.x];
	__syncthreads();
	//--------------
	for (int offset=blockDim.x/2; offset>0; offset=offset/2) 
	{	if (threadIdx.x < offset)
		{	shared[threadIdx.x] = fminf(shared[threadIdx.x],
			   shared[threadIdx.x+offset]);
		}
		__syncthreads();
	}
        if (threadIdx.x == 0) gfMin[0] = shared[0];
}

static __global__ void mGFindMax1D (float* gfMax)
{
	extern __shared__ float shared[];
	shared[threadIdx.x] = gfMax[threadIdx.x];
	__syncthreads();
	//--------------
	for (int offset=blockDim.x/2; offset>0; offset=offset/2)
	{	if (threadIdx.x < offset)
		{	shared[threadIdx.x] = fmaxf(shared[threadIdx.x],
			   shared[threadIdx.x+offset]);
		}
		__syncthreads();
	}
        if (threadIdx.x == 0) gfMax[0] = shared[0];
}

GFindMinMax2D::GFindMinMax2D(void)
{
        m_aBlockDim.x = 512;
	m_aBlockDim.y = 1;
	m_aGridDim.x = 512;
	m_aGridDim.y = 1;
	m_gfBuf = 0L;
}

GFindMinMax2D::~GFindMinMax2D(void)
{
	this->Clean();
}

void GFindMinMax2D::Clean(void)
{
	if(m_gfBuf == 0L) return;
	cudaFree(m_gfBuf);
	m_gfBuf = 0L;
}

void GFindMinMax2D::SetSize(int* piImgSize, bool bPadded)
{
	this->Clean();
	//------------
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

float GFindMinMax2D::DoMin
(	float* gfImg, 
	bool bSync, 
	cudaStream_t stream
)
{	int iShmBytes = sizeof(float) * m_aBlockDim.x;
	mGFindMin2D<<<m_aGridDim, m_aBlockDim, iShmBytes, stream>>>(gfImg,
	   m_aiImgSize[0], m_aiImgSize[1], m_iPadX, m_gfBuf);
        mGFindMin1D<<<1, m_aGridDim, iShmBytes, stream>>>(m_gfBuf);
	//---------------------------------------------------------
	if(bSync || stream == 0)
	{	float fMin = this->GetResult(); 
		return fMin;
	}
	else return 0.0f;
}

float GFindMinMax2D::DoMax
(       float* gfImg,
        bool bSync,
        cudaStream_t stream
)
{       int iShmBytes = sizeof(float) * m_aBlockDim.x;
        mGFindMax2D<<<m_aGridDim, m_aBlockDim, iShmBytes, stream>>>(gfImg,
           m_aiImgSize[0], m_aiImgSize[1], m_iPadX, m_gfBuf);
        mGFindMax1D<<<1, m_aGridDim, iShmBytes, stream>>>(m_gfBuf);
        //----------------------------------------------------------
	if(bSync || stream == 0)
        {       float fMax = this->GetResult();
                return fMax;
        }
        else return 0.0f;
}

float GFindMinMax2D::GetResult(void)
{
	float fRes = 0.0f;
	cudaMemcpy(&fRes, m_gfBuf, sizeof(float), cudaMemcpyDefault);
	return fRes;
}	


void GFindMinMax2D::Test(float* gfImg)
{
	int iPixels = m_iPadX * m_aiImgSize[1];
	float* pfImg = new float[iPixels];
	cudaMemcpy(pfImg, gfImg, iPixels * sizeof(float), cudaMemcpyDefault);
	//-------------------------------------------------------------------
	float fMin = pfImg[0];
	float fMax = pfImg[0];
	for(int y=0; y<m_aiImgSize[1]; y++)
	{	int i = y * m_iPadX;
		for(int x=0; x<m_aiImgSize[0]; x++)
		{	float fVal = pfImg[i+x];
			if(fMin > fVal) fMin = fVal;
			else if(fMax < fVal) fMax = fVal;
		}
	}
	printf("GFindMinMax2D: CPU Res: %.3e  %.3e\n", fMin, fMax); 
	delete[] pfImg;
}

