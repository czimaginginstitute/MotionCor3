#include "CFindCTFInc.h"
#include "../Util/CUtilInc.h"
#include <CuUtilFFT/GFFT1D.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::GCTFFind;

static __global__ void mGRemove1D
(	float* gfBackground,
	int iBackgroundSize,
	int iBackgroundStart,
	float* gfSpectrum,
	int iSpectrumSize
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= iSpectrumSize) return;
	//----------------------------
	if(i < iBackgroundStart)
	{	gfSpectrum[i] = 0.0f;
		return;
	}
	//----------
	if(i >= iBackgroundSize) i = iBackgroundSize - 1;
	gfSpectrum[i] -= gfBackground[i];
}

static __global__ void mGRemove2D
(   float* gfBackground,
	int iBackgroundSize,
    int iBackgroundStart,
    float* gfSpectrum,
    int iSizeY
)
{   int y = blockIdx.y * blockDim.y + threadIdx.y;
    if(y >= iSizeY) return;
    int i = y * gridDim.x + blockIdx.x;
    //---------------------------------
    y = y - iSizeY / 2;
    float fR = sqrtf(blockIdx.x * blockIdx.x + y * y);
    //------------------------------------------------
    if(fR < iBackgroundStart)
    {	gfSpectrum[i] = 0.0f;
     	return;
    }
    else if(fR >= (iBackgroundSize - 1))
    {	gfSpectrum[i] -= gfBackground[iBackgroundSize-1];
		return;
    }
	//----------
    int iR = (int)fR;
    float fBgVal = gfBackground[iR] * (iR+1-fR)
        + gfBackground[iR+1] * (fR - iR);
    gfSpectrum[i] -= fBgVal;
}

GBackground1D::GBackground1D(void)
{
	m_gfBackground = 0L;
}

GBackground1D::~GBackground1D(void)
{
}

void GBackground1D::SetBackground
(	float* gfBackground,
	int iStart,
	int iSize
)
{	m_gfBackground = gfBackground;
	m_iStart = iStart;
	m_iSize = iSize;
}

void GBackground1D::Remove1D(float* gfSpectrum, int iSize)
{
	if(m_gfBackground == 0L) return;
	//------------------------------
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(iSize / aBlockDim.x+1, 1);
	mGRemove1D<<<aGridDim, aBlockDim>>>
	(  m_gfBackground, m_iSize, m_iStart,
       gfSpectrum, iSize
	);
}

void GBackground1D::Remove2D(float* gfSpectrum, int* piSize)
{
     if(m_gfBackground == 0L) return;
     //------------------------------
     dim3 aBlockDim(1, 512);
     dim3 aGridDim(piSize[0], piSize[1]/aBlockDim.y+1);
     mGRemove2D<<<aGridDim, aBlockDim>>>
     (  m_gfBackground, m_iSize, m_iStart,
        gfSpectrum, piSize[1]
     );
}

void GBackground1D::DoIt
(	float* pfSpectrum,
	int iSize
)
{    m_iSize = iSize;
     //--------------
     size_t tBytes = sizeof(float) * m_iSize;
     float* pfRawSpectrum = new float[m_iSize];
     cudaMemcpy(pfRawSpectrum, pfSpectrum, tBytes, cudaMemcpyDefault);
     //---------------------------------------------------------------
	 float* pfBackground = new float[m_iSize];
	 memset(pfBackground, 0, sizeof(float) * m_iSize);
	 //-----------------------------------------------
     m_iStart = mFindStart(pfRawSpectrum);
	 //-----------------------------------
	 int iWinSize = m_iStart / 2 * 2 - 1;
	 //if(iWinSize > m_iStart) iWinSize = m_iStart / 2 * 2 - 1;
	 //------------------------------------------------------
	 for(int i=m_iStart; i<m_iSize; i++)
	 {	float fMean = 0.0f;
		for(int j=0; j<iWinSize; j++)
		{	int k = i + j - iWinSize / 2;
			if(k >= m_iSize) k = 2 * m_iSize - 1 - k;
			fMean += pfRawSpectrum[k];
		}
		pfBackground[i] = fMean / iWinSize;
		/*
		printf("%4d  %.4e  %.4e  %.4e\n", i, pfRawSpectrum[i],
			pfBackground[i], pfRawSpectrum[i] - pfBackground[i]);
		*/
	 }
	 //------------------------------------
     if(m_gfBackground != 0L) cudaFree(m_gfBackground);
     cudaMalloc(&m_gfBackground, tBytes);
     cudaMemcpy(m_gfBackground, pfBackground, tBytes, cudaMemcpyDefault);
     delete[] pfBackground;
	 delete[] pfRawSpectrum;
}

int GBackground1D::mFindStart(float* pfSpectrum)
{
     int iStart = m_iSize / 5;
	 int iEnd = m_iSize - iStart;
     double dX = 0, dY = 0, dX2 = 0, dXY = 0;
     for(int i=iStart; i<iEnd; i++)
     {    dX += i;
          dX2 += (i * i);
          dY += pfSpectrum[i];
          dXY += (i * pfSpectrum[i]);
     }
     int iCount = iEnd - iStart;
     dX /= iCount;
     dY /= iCount;
     dX2 /= iCount;
     dXY /= iCount;
     double dA = (dX * dY - dXY) / (dX * dX - dX2);
     double dB = dY - dA * dX;
     //-----------------------
     for(int i=2; i<m_iSize; i++)
     {    dY = dA * i + dB;
          if(dY < pfSpectrum[i]) continue;
          iStart = i;
          break;
     }
     return iStart;
}
