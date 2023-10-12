#include "CMotionDeconInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2;
using namespace MotionCor2::MotionDecon;

static __device__ float mGCalcDirSinc
(	int iCmpY,
	float fShiftX,
	float fShiftY 
)
{	float fShift = sqrtf(fShiftX * fShiftX + fShiftY * fShiftY);
	//fShift *= 2.0f; // 1 pixel shift smears 2 pixels.
	if(fShift <= 0.1f) return 1.0f;
	fShift += 1.0f;
	//-------------
	// slope of the line perpendicular to the motion vector
	//-----------------------------------------------------
	float fSlope = -fShiftX / (fShiftY + (float)1e-20);
	//-------------------------------------------------
	float fDist = 0.0f;
	if(fabs(fSlope) > (float)1e5)
	{	fDist = blockIdx.x;
	}
	else
	{	int y = blockIdx.y * blockDim.y + threadIdx.y;
		if(y > (iCmpY / 2)) y -= iCmpY;
		fDist = fabs(fSlope * blockIdx.x - y) 
			/ sqrtf(fSlope * fSlope + 1.0f);
	}
	if(fDist < 0.1f) return 1.0f;
	//---------------------------
	float fPI = 3.141592654f;
	fDist = fDist / (2 * (gridDim.x - 1));
	if((fShift * fDist) > 1.0f) return 0.0f;
	float fSinc = sinf(fPI * fShift * fDist) / (fPI * fShift * fDist);
	return fSinc;
}

static __device__ float mCalcSinc
(	int iCmpY,
	float fShift
)
{	fShift *= 2.0f; // 1 pixel shift affects 2.
	if(fShift < 0.5f) return 1.0f;
	//----------------------------
	float fX = blockIdx.x * 0.5f / (gridDim.x - 1.0f);
	float fY = blockIdx.y * blockDim.y + threadIdx.y;
	fY = fY * 1.0f / iCmpY;
	if(fY > 0.5f) fY -= 1.0f;
	float fR = sqrtf(fX * fX + fY * fY);
	if(fR < 0.1f) return 1.0f;
	//------------------------
	float fPI = 3.141592654f;
	float fFact = fShift * fR;
	if(fFact >= 1.0) return 0.0f;
	float fSinc = sinf(fPI * fFact) / (fPI * fFact);
	return fSinc;
}

static __global__ void mGSumSinc2
(	float fShiftX,
	float fShiftY,
	float* gfSinc2Sum,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
        int i = y * gridDim.x + blockIdx.x;
	if(y >= iCmpY || i == 0) return;
	//------------------------------
	float fSinc = mGCalcDirSinc(iCmpY, fShiftX, fShiftY);
	gfSinc2Sum[i] += (fSinc * fSinc);
}
/*------------------------------------
static __global__ void mGDeconFrame
(	float fShiftX,
	float fShiftY,
	float* gfSinc2Sum,
	cufftComplex* gCmpFrm,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
        int i = y * gridDim.x + blockIdx.x;
        if(y >= iCmpY || i == 0) return;
	//------------------------------
	float fSinc = mGCalcSinc(iCmpY, fShiftX, fShiftY);
	fSinc /= (gfSinc2Sum[i] + (float)1e-20);
	gCmpFrm[i].x *= fSinc;
	gCmpFrm[i].y *= fSinc;
}
-----------------------------------------*/

static __global__ void mGDirWeightFrame
(	float fShiftX,
	float fShiftY,
	float* gfSinc2Sum,
	cufftComplex* gCmpFrm,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int i = y * gridDim.x + blockIdx.x;
	if(y >= iCmpY || i == 0) return;
	//------------------------------
	float fSinc = mGCalcDirSinc(iCmpY, fShiftX, fShiftY);
	gCmpFrm[i].x *= fSinc;
	gCmpFrm[i].y *= fSinc;
}

static __global__ void mGWeightFrame
(	float fShift,
	cufftComplex* gCmpFrm,
	int iCmpY
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	int i = y * gridDim.x + blockIdx.x;
	if(y >= iCmpY || i == 0) return;
	//------------------------------
	float fSinc = mCalcSinc(iCmpY, fShift);
	gCmpFrm[i].x *= fSinc;
	gCmpFrm[i].y *= fSinc;
} 

GDeconFrame::GDeconFrame(void)
{	
	m_gfSinc2Sum = 0L;
	m_pfInFrmShifts = 0L;
}

GDeconFrame::~GDeconFrame(void)
{
	if(m_pfInFrmShifts != 0L) delete[] m_pfInFrmShifts;
	this->CleanGpu();
}

void GDeconFrame::CleanGpu(void)
{
	if(m_gfSinc2Sum != 0L) cudaFree(m_gfSinc2Sum);
	m_gfSinc2Sum = 0L;
}

void GDeconFrame::CalcSinc2Sum
(	float* pfInFrmShifts,
	int iNumFrames,
	int* piCmpSize
)
{	m_iNumFrames = iNumFrames;
	m_aiCmpSize[0] = piCmpSize[0];
	m_aiCmpSize[1] = piCmpSize[1];
	//----------------------------
	if(m_pfInFrmShifts != 0L) delete[] m_pfInFrmShifts;
	m_pfInFrmShifts = new float[2 * m_iNumFrames];
	memcpy(m_pfInFrmShifts, pfInFrmShifts, 
		sizeof(float) * 2 * m_iNumFrames); 	
	//----------------------------------------
	this->CleanGpu();
	int iCmpSize = m_aiCmpSize[0] * m_aiCmpSize[1];
	size_t tBytes = sizeof(float) * iCmpSize;
	cudaMalloc(&m_gfSinc2Sum, tBytes);
	cudaMemset(m_gfSinc2Sum, 0, tBytes);
	//----------------------------------
	dim3 aBlockDim(1, 512);
        dim3 aGridDim(m_aiCmpSize[0], 1);
        aGridDim.y = m_aiCmpSize[1] / aBlockDim.y + 1;
	//--------------------------------------------
	for(int i=0; i<iNumFrames; i++)
	{	int j = 2 * i;
		mGSumSinc2<<<aGridDim, aBlockDim>>>
		(  m_pfInFrmShifts[j], m_pfInFrmShifts[j+1], 
		   m_gfSinc2Sum, m_aiCmpSize[1]
		);
	} 
}

void GDeconFrame::DirWeightFrame
(	int iFrame,
	cufftComplex* pCmpFrm,
	bool bGpu 
)
{	cufftComplex* gCmpFrm = mH2D(pCmpFrm, bGpu, m_aiCmpSize);
	//-------------------------------------------------------
	dim3 aBlockDim(1, 512);
        dim3 aGridDim(m_aiCmpSize[0], 1);
        aGridDim.y = m_aiCmpSize[1] / aBlockDim.y + 1;
	//--------------------------------------------
	float fShiftX = m_pfInFrmShifts[2 * iFrame];
	float fShiftY = m_pfInFrmShifts[2 * iFrame + 1];
	//----------------------------------------------
	mGDirWeightFrame<<<aGridDim, aBlockDim>>>
	(  fShiftX, fShiftY, m_gfSinc2Sum, 
	   gCmpFrm, m_aiCmpSize[1]
	);
	//------------------------
	mD2H(gCmpFrm, pCmpFrm, bGpu, m_aiCmpSize);
	if(!bGpu) cudaFree(gCmpFrm);
}

void GDeconFrame::WeightFrame
(	float fShift,
	cufftComplex* pCmpFrm,
	bool bGpu,
	int* piCmpSize
)
{	cufftComplex* gCmpFrm = mH2D(pCmpFrm, bGpu, piCmpSize);
        //-----------------------------------------------------
	dim3 aBlockDim(1, 512);
        dim3 aGridDim(piCmpSize[0], 1);
        aGridDim.y = piCmpSize[1] / aBlockDim.y + 1;
        //------------------------------------------
	mGWeightFrame<<<aGridDim, aBlockDim>>>
        (  fShift, gCmpFrm, piCmpSize[1]
        );
        //------------------------------
	mD2H(gCmpFrm, pCmpFrm, bGpu, piCmpSize);
        if(!bGpu) cudaFree(gCmpFrm);
}

cufftComplex* GDeconFrame::mH2D
(	cufftComplex* pCmpFrm,
	bool bGpu,
	int* piCmpSize
)
{	if(bGpu) return pCmpFrm;
	//----------------------
	size_t tBytes = sizeof(cufftComplex);
	tBytes *= (piCmpSize[0] * piCmpSize[1]);
	cufftComplex* gCmpFrm = 0L;
	cudaMalloc(&gCmpFrm, tBytes);
	cudaMemcpy(gCmpFrm, pCmpFrm, tBytes, cudaMemcpyHostToDevice);
	return gCmpFrm;
}

void GDeconFrame::mD2H
(	cufftComplex* gCmpFrm,
	cufftComplex* pCmpFrm,
	bool bGpu,
	int* piCmpSize
)
{	if(bGpu) return;
	//--------------
	size_t tBytes = sizeof(cufftComplex);
        tBytes *= (piCmpSize[0] * piCmpSize[1]);
	cudaMemcpy(pCmpFrm, gCmpFrm, tBytes, cudaMemcpyDeviceToHost);
}
	
