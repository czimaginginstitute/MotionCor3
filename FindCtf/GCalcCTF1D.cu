#include "CFindCtfInc.h"
#include <CuUtilFFT/GFFT2D.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>

using namespace MotionCor2::FindCtf;

//-----------------------------------------------------------------------------
// 1. Calculate theoretical CTF given the CTF parameters.
//    0: wavelength in pixel
//    1: Cs in pixel
//    2: amplitude contrast contributed phase
//-----------------------------------------------------------------------------
static __device__ __constant__ float s_gfCtfParam[2];

static __global__ void mGCalculate
(	float fDefocus,  // in pixel
	float fExtPhase, // extra phase from amp contrast and phase plate
	float* gfCTF1D,
	int iCmpSize
)
{	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i >= iCmpSize) return;
	//-----------------------
	float fs2 = (i * 0.5f) / (iCmpSize - 1.0f);
	fs2 = fs2 * fs2;
	float fw2 = s_gfCtfParam[0] * s_gfCtfParam[0];
	fw2 = fExtPhase + 3.141592654f * s_gfCtfParam[0] * fs2
	   * (fDefocus - 0.5f * fw2 * fw2 * s_gfCtfParam[1]);
	//---------------------------------------------------
	gfCTF1D[i] = -sinf(fw2);
}

GCalcCTF1D::GCalcCTF1D(void)
{
}

GCalcCTF1D::~GCalcCTF1D(void)
{
}

void GCalcCTF1D::SetParam(CCtfParam* pCtfParam)
{
	float afCtfParam[2] = {0.0f};
	afCtfParam[0] = pCtfParam->m_fWaveLen;
	afCtfParam[1] = pCtfParam->m_fCs;
	cudaMemcpyToSymbol(s_gfCtfParam, afCtfParam, sizeof(float) * 2);
	m_fAmpPhase = pCtfParam->m_fAmpPhaseShift;
}

void GCalcCTF1D::DoIt
(	float fDefocus,   // in pixel
	float fExtPhase,  // phase in radian from phase plate
	float* gfCTF1D,
	int iCmpSize
)
{	dim3 aBlockDim(512, 1);
	dim3 aGridDim(1, 1);
	aGridDim.x = (iCmpSize + aBlockDim.x - 1) / aBlockDim.x;
	float fAddPhase = m_fAmpPhase + fExtPhase;
	mGCalculate<<<aGridDim, aBlockDim>>>(fDefocus, fAddPhase, 
	   gfCTF1D, iCmpSize);
}
