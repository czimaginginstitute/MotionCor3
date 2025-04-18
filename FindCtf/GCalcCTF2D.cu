#include "CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::FindCtf;

//--------------------------------------------------------------
// 0: wavelength in pixel
// 1: Cs in pixel
// 2: Extra phase shift from amplitude contrast and phase plate.
//--------------------------------------------------------------
static __device__ __constant__ float s_gfCtfParam[2];

static __global__ void mGCalculate
(	float fDfMean,
	float fDfSigma,
	float fAzimuth,
	float fExtPhase,
	float* gfCTF2D,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iCmpY) return;
	//--------------------------------------------
	float fX = blockIdx.x * 0.5f / (gridDim.x - 1);
	float fY = (y - iCmpY / 2) / (float)iCmpY;
	float fS2 = fX * fX + fY * fY;
	float fW2 = s_gfCtfParam[0] * s_gfCtfParam[0];
	//--------------------------------------------
	fX = atanf(fY / (fX + (float)1e-30));
	fX = fDfMean + fDfSigma * cosf(2.0f * (fX - fAzimuth));
	//-----------------------------------------------------
	fX = -sinf(fExtPhase + 3.1415926f * s_gfCtfParam[0] * fS2
	   * (fX - 0.5f * fW2 * fS2 * s_gfCtfParam[1]));
	//----------------------------------------------
	gfCTF2D[y * gridDim.x + blockIdx.x] = fX * fX;
}

//------------------------------------------------------------------------------
// Embed the estimated CTF on the right half of the full spectrum for the 
// purpose of diagnosis. 
// 1. The gfFullSpect has the size of iNx and iNy. 
// 2. gfHalfSpect has the size of iCmpX and iCmpY.
// 3. iCmpX = iNx / 2 + 1, iCmpY = iNy.
// 4. The right half region include [iNx/2, iNx-1] and [0, iNy-1].
// 5. The DC is at (iNx/2, iNy/2)
//------------------------------------------------------------------------------
static __global__ void mGEmbedCtf
(	float* gfCtf2D, int iCmpY,
	float fMinFreq, float fMaxFreq,
	float fMean, float fGain,
	float* gfFullSpect, int iFullSpectX
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iCmpY) return;
	//-----------------
	float fY = (y - iCmpY * 0.5f) / iCmpY;
	float fX = (blockIdx.x - (float)gridDim.x) * 0.5f / gridDim.x;
	fX = sqrtf(fX * fX + fY * fY);
	if(fX < fMinFreq || fX > fMaxFreq) return;
	//----------------------------------------------
	// fX is negative frequency, apply symmetry here
	//----------------------------------------------
	int iX = gridDim.x - blockIdx.x;
	int iY = (iCmpY - y) % iCmpY;
	fY = gfCtf2D[iY * (gridDim.x + 1) + iX];
	fY = (fY - 0.5f) * fGain + fMean;
	//--------------------------------------------------------
	// CTF is embededd on the right half of the full spectrum.
	// Therefore, there is a (gridDim.x = iNx / 2) offset in
	// the horizontal axis.
	//--------------------------------------------------------
	gfFullSpect[y * iFullSpectX + blockIdx.x] = fY;
}

GCalcCTF2D::GCalcCTF2D(void)
{
}

GCalcCTF2D::~GCalcCTF2D(void)
{
}

void GCalcCTF2D::SetParam(CCtfParam* pCtfParam)
{
	float afCtfParam[2] = {0.0f};
	afCtfParam[0] = pCtfParam->m_fWaveLen;
	afCtfParam[1] = pCtfParam->m_fCs;
	cudaMemcpyToSymbol(s_gfCtfParam, afCtfParam, sizeof(float) * 2);
	//-----------------
	m_fAmpPhase = pCtfParam->m_fAmpPhaseShift;
}

void GCalcCTF2D::DoIt
(	float fDfMin,   float fDfMax, 
	float fAzimuth, float fExtPhase, 
	float* gfCTF2D, int* piCmpSize
)
{	float fDfMean = 0.5f * (fDfMin + fDfMax);
	float fDfSigma = 0.5f * (fDfMax - fDfMin);
	//-----------------
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(piCmpSize[0], 1);
	aGridDim.y = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	//-----------------
	float fAddPhase = m_fAmpPhase + fExtPhase;
	mGCalculate<<<aGridDim, aBlockDim>>>(fDfMean, fDfSigma, fAzimuth,
	   fAddPhase, gfCTF2D, piCmpSize[1]);
}

void GCalcCTF2D::DoIt(CCtfParam* pCtfParam, float* gfCtf2D, int* piCmpSize)
{
	this->SetParam(pCtfParam);
	this->DoIt(pCtfParam->m_fDfMin, pCtfParam->m_fDfMax,
	   pCtfParam->m_fAstAng, pCtfParam->m_fExtPhase,
	   gfCtf2D, piCmpSize);
}

void GCalcCTF2D::EmbedCtf
(	float* gfCtf2D, int* piCmpSize, 
	float fMinFreq, float fMaxFreq,
	float fMean, float fGain, 
	float* gfFullSpect, bool bPadded
)
{	int iHalfX = piCmpSize[0] - 1;
	int iFullSpectX = iHalfX * 2;
	if(bPadded) iFullSpectX = piCmpSize[0] * 2;
	//-----------------
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(iHalfX, 1);
	aGridDim.y = (piCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	//-----------------
	mGEmbedCtf<<<aGridDim, aBlockDim>>>(gfCtf2D, piCmpSize[1],
	   fMinFreq, fMaxFreq, fMean, fGain, 
	   gfFullSpect, iFullSpectX);
}
