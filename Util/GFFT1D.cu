#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <math.h>
#include <stdio.h>

using namespace MotionCor2::Util;

static __global__ void mGMultiply
(	cufftComplex* gCmpLines, 
	int iCmpSize,
	float fFactor
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= iCmpSize) return;
	//-----------------------
	int j = blockIdx.y * iCmpSize + i;
	gCmpLines[j].x *= fFactor;
	gCmpLines[j].y *= fFactor;
}

GFFT1D::GFFT1D(void)
{
	m_cufftPlan = 0;
	m_iFFTSize = 0;
	m_iNumLines = 0;
	m_cufftType = CUFFT_R2C;
}

GFFT1D::~GFFT1D(void)
{
	this->DestroyPlan();
}

void GFFT1D::DestroyPlan(void)
{
	if(m_cufftPlan == 0) return;
	cufftDestroy(m_cufftPlan);
	m_cufftPlan = 0;
	m_iFFTSize = 0;
	m_iNumLines = 0;
}

void GFFT1D::CreatePlan(int iFFTSize, int iNumLines, bool bForward)
{
	cufftType fftType = bForward ? CUFFT_R2C : CUFFT_C2R;
	if(fftType != m_cufftType) this->DestroyPlan();
	else if(m_iFFTSize != iFFTSize) this->DestroyPlan();
	else if(m_iNumLines != iNumLines) this->DestroyPlan();
	if(m_cufftPlan != 0) return;
	//--------------------------
	m_cufftType = fftType;
	m_iFFTSize = iFFTSize;
	m_iNumLines = iNumLines;
	//----------------------
	cufftResult res = cufftPlan1d
	( &m_cufftPlan, m_iFFTSize, m_cufftType, m_iNumLines
	);
}

void GFFT1D::Forward
(	float* gfPadLines,
	bool bNorm
)
{	cufftResult res = cufftExecR2C
	( m_cufftPlan, (cufftReal*)gfPadLines,
	  (cufftComplex*)gfPadLines
	);
	if(!bNorm) return;
	//----------------
	int iCmpSize = m_iFFTSize / 2 + 1;
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(iCmpSize / aBlockDim.x + 1, m_iNumLines);
	float fFactor = 1.0f / m_iFFTSize;
	//--------------------------------
	mGMultiply<<<aGridDim, aBlockDim>>>
	( (cufftComplex*)gfPadLines,
	  iCmpSize, fFactor
	);
}

void GFFT1D::Inverse(cufftComplex* gCmpLines)
{	
	cufftResult res = cufftExecC2R
	( m_cufftPlan, gCmpLines, (cufftReal*)gCmpLines
	);
}

