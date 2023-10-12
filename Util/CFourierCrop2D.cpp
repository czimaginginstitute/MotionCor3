#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::Util;

CFourierCrop2D::CFourierCrop2D(void)
{
}

CFourierCrop2D::~CFourierCrop2D(void)
{
	this->Clear();
}

void CFourierCrop2D::Clear(void)
{	
	m_aCufft2D.DestroyPlan();
}

void CFourierCrop2D::Setup(int* piCmpSize, float fBin)
{
	m_aiInCmpSize[0] = piCmpSize[0];
	m_aiInCmpSize[1] = piCmpSize[1];
	//------------------------------
	GFourierResize2D::GetBinnedCmpSize(m_aiInCmpSize, 
	   fBin, m_aiOutCmpSize);
	m_aCufft2D.CreateInversePlan(m_aiOutCmpSize, true);
}

void CFourierCrop2D::Setup(int* piCmpSizeIn, int* piCmpSizeOut)
{
	memcpy(m_aiInCmpSize, piCmpSizeIn, sizeof(int) * 2);
	memcpy(m_aiOutCmpSize, piCmpSizeOut, sizeof(int) * 2);
	m_aCufft2D.CreateInversePlan(m_aiOutCmpSize, true);
}

void CFourierCrop2D::DoIt
(	cufftComplex* gCmp,
	cufftComplex* gCmpBuf,
	float* gfImg,
	cudaStream_t stream
)
{	GFourierResize2D fourierResize;
	fourierResize.DoIt
	( gCmp, m_aiInCmpSize, gCmpBuf, m_aiOutCmpSize, false, stream);
	///------------------------------------------------------------
	m_aCufft2D.Inverse(gCmpBuf, stream);
	int aiPadSize[] = {m_aiOutCmpSize[0] * 2, m_aiOutCmpSize[1]};
	float* gfPadImg = reinterpret_cast<float*>(gCmpBuf);
	//--------------------------------------------------
	GPad2D aGPad2D;
	aGPad2D.Unpad(gfPadImg, aiPadSize, gfImg, stream);
}

void CFourierCrop2D::GetCmpSize
(	int* piCmpSizeIn, 
	float fBin,
	int* piCmpSizeOut
)
{	GFourierResize2D::GetBinnedCmpSize(piCmpSizeIn, 
	   fBin, piCmpSizeOut);
}

void CFourierCrop2D::GetImgSize
(	int* piCmpSizeIn,
	float fBin,
	int* piImgSizeOut
)
{	GFourierResize2D::GetBinnedCmpSize(piCmpSizeIn, fBin, piImgSizeOut);
	piImgSizeOut[0] = (piImgSizeOut[0] - 1) * 2;
}
