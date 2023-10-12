#include "CUtilInc.h"
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <Util/Util_Time.h>
#include <assert.h>

using namespace MotionCor2::Util;

CCufft2D::CCufft2D(void)
{
	m_aPlan = 0;
	m_iFFTx = 0;
	m_iFFTy = 0;
	m_aType = CUFFT_R2C;
}

CCufft2D::~CCufft2D(void)
{
     this->DestroyPlan();
}

void CCufft2D::CreateForwardPlan(int* piSize, bool bPad)
{
	int iFFTx = bPad? (piSize[0] / 2 - 1) * 2 : piSize[0];
	if(iFFTx == m_iFFTx && piSize[1] == m_iFFTy 
		&& m_aType == CUFFT_R2C) return;
	//--------------------------------------
	this->DestroyPlan();
	m_iFFTx = iFFTx;
	m_iFFTy = piSize[1];
	m_aType = CUFFT_R2C;
     	cufftResult res = cufftPlan2d(&m_aPlan, m_iFFTy, m_iFFTx, m_aType);
	//-----------------------------------------------------------------
	const char* pcFormat = "CCufft2D::CreateForwardPlan: %s\n";
	mCheckError(&res, pcFormat);
}

void CCufft2D::CreateInversePlan(int* piSize, bool bCmp)
{
	int iFFTx = bCmp? (piSize[0] - 1) * 2 : piSize[0];
	if(iFFTx == m_iFFTx && piSize[1] == m_iFFTy
		&& m_aType == CUFFT_C2R) return;
	//--------------------------------------
	this->DestroyPlan();
	m_iFFTx = iFFTx;
	m_iFFTy = piSize[1];
	m_aType = CUFFT_C2R;
	cufftResult res = cufftPlan2d(&m_aPlan, m_iFFTy, m_iFFTx, m_aType);
	//-----------------------------------------------------------------
	const char* pcFormat = "CCufft2D::CreateInversePlan: %s\n";
	mCheckError(&res, pcFormat);
}

void CCufft2D::DestroyPlan(void)
{
	if(m_aPlan == 0) return;
	cufftDestroy(m_aPlan);
	m_aPlan = 0;
	m_iFFTx = 0;
	m_iFFTy = 0;
	Util::CheckCudaError("CCufft2D::DestroyPlan:62");
}

bool CCufft2D::Forward(float* gfPadImg, cufftComplex* gCmpImg,
	bool bNorm, cudaStream_t stream)
{
	const char* pcFormat = "CCufft2D::Forward: %s\n\n";
	cufftSetStream (m_aPlan, stream);
	cufftResult res = cufftExecR2C(m_aPlan, 
	   (cufftReal*)gfPadImg, gCmpImg);
	if(mCheckError(&res, pcFormat)) return false;
	else if(!bNorm) return true;
	//--------------------------
	int aiCmpSize[] = {m_iFFTx / 2 + 1, m_iFFTy};
	float fFactor = (float)(1.0 / m_iFFTx / m_iFFTy);
	GFFTUtil2D aGFFTUtil;
        aGFFTUtil.Multiply(gCmpImg, aiCmpSize, fFactor, stream);
	return true;
}

bool CCufft2D::Forward(float* gfPadImg, bool bNorm, cudaStream_t stream)
{
        bool bSuccess = this->Forward(gfPadImg, 
	   (cufftComplex*)gfPadImg, bNorm, stream);
	return bSuccess;
}

cufftComplex* CCufft2D::ForwardH2G
(	float* pfImg, 
	bool bNorm
)
{	int aiImgSize[2] = {m_iFFTx, m_iFFTy};
	//------------------------------------
	CPad2D aPad2D;
	float* gfPad = aPad2D.GPad(pfImg, aiImgSize);
	//-------------------------------------------
	bool bSuccess = this->Forward(gfPad, bNorm);
	if(bSuccess) return (cufftComplex*)gfPad;
	//---------------------------------------
	cudaFree(gfPad);
	return 0L;
}
	
bool CCufft2D::Inverse(cufftComplex* gCmp, float* gfPadImg, 
	cudaStream_t stream)
{
	const char* pcFormat = "CCufft2D::Inverse: %s\n";
	//-----------------------------------------------
        cufftSetStream (m_aPlan, stream);
	cufftResult res = cufftExecC2R(m_aPlan, gCmp, (cufftReal*)gfPadImg);
	if(mCheckError(&res, pcFormat)) return false;
	else return true;
}

bool CCufft2D::Inverse(cufftComplex* gCmp, cudaStream_t stream)
{
	bool bSuccess = this->Inverse(gCmp, (float*)gCmp, stream);
	return bSuccess;
}

float* CCufft2D::InverseG2H(cufftComplex* gCmp)
{
	if(!this->Inverse(gCmp)) return 0L;
	//---------------------------------
	CPad2D aPad2D;
	int aiPadSize[] = {(m_iFFTx / 2 + 1) * 2, m_iFFTy};
	float* pfImg = aPad2D.CUnpad((float*)gCmp, aiPadSize);
	return pfImg;
}	

void CCufft2D::SubtractMean(cufftComplex* gComplex)
{
	cudaMemset(gComplex, 0, sizeof(cufftComplex));
}

bool CCufft2D::mCheckError(cufftResult* pResult, const char* pcFormat)
{
        if(*pResult == CUFFT_SUCCESS) return false;
	//-----------------------------------------
	const char* pcErr = mGetErrorEnum(*pResult);	
	fprintf(stderr, pcFormat, pcErr);
	cudaDeviceReset();
	assert(0);
        return true;
}

const char* CCufft2D::mGetErrorEnum(cufftResult error)
{
	switch (error)
    	{	case CUFFT_SUCCESS:
            	return "CUFFT_SUCCESS";
		//---------------------
		case CUFFT_INVALID_PLAN:
            	return "CUFFT_INVALID_PLAN";
		//--------------------------
        	case CUFFT_ALLOC_FAILED:
            	return "CUFFT_ALLOC_FAILED";
		//--------------------------
        	case CUFFT_INVALID_TYPE:
            	return "CUFFT_INVALID_TYPE";
		//--------------------------
        	case CUFFT_INVALID_VALUE:
            	return "CUFFT_INVALID_VALUE";
		//---------------------------
        	case CUFFT_INTERNAL_ERROR:
           	return "CUFFT_INTERNAL_ERROR";
		//----------------------------
        	case CUFFT_EXEC_FAILED:
           	return "CUFFT_EXEC_FAILED";
		//-------------------------
        	case CUFFT_SETUP_FAILED:
            	return "CUFFT_SETUP_FAILED";
		//--------------------------
        	case CUFFT_INVALID_SIZE:
            	return "CUFFT_INVALID_SIZE";
		//--------------------------
        	case CUFFT_UNALIGNED_DATA:
            	return "CUFFT_UNALIGNED_DATA";
    	}
   	return "<unknown>";
}

