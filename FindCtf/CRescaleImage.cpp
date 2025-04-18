#include "CFindCtfInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <cuda.h>
#include <cufft.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2;
using namespace MotionCor2::FindCtf;
namespace DU = MotionCor2::DataUtil;

CRescaleImage::CRescaleImage(void)
{
	m_gfPadImgN = 0L;
	m_fPixSizeN = 1.0f;
	memset(m_aiImgSizeN, 0, sizeof(m_aiImgSizeN));
	memset(m_aiPadSizeN, 0, sizeof(m_aiPadSizeN));
}

CRescaleImage::~CRescaleImage(void)
{
}

void CRescaleImage::DoIt
(	float* gfImg, int* piImgSize,
	DU::CDataPackage* pPackage
)
{	float fPixSize = pPackage->m_pAlnSums->m_fPixSize;	
	//---------------------------
	m_fBinning = 1.25f / fPixSize;
	if(m_fBinning <= 1) m_fBinning = 1.0f;
	m_fPixSizeN = fPixSize * m_fBinning;
	//---------------------------
	m_aiImgSizeN[0] = (int)(piImgSize[0] / m_fBinning + 0.5f);
	m_aiImgSizeN[1] = (int)(piImgSize[1] / m_fBinning + 0.5f);
	m_aiImgSizeN[0] = m_aiImgSizeN[0] / 2 * 2;
	m_aiImgSizeN[1] = m_aiImgSizeN[1] / 2 * 2;
	//---------------------------
	m_aiPadSizeN[0] = (m_aiImgSizeN[0] / 2 + 1) * 2;
	m_aiPadSizeN[1] = m_aiImgSizeN[1];
	//---------------------------
	int iBytes = sizeof(float) * m_aiPadSizeN[0] * m_aiPadSizeN[1];
	if(m_gfPadImgN != 0L) cudaFree(m_gfPadImgN);
	cudaMalloc(&m_gfPadImgN, iBytes);
	//---------------------------
	Util::GPad2D pad2D;
	if(m_fBinning == 1)
	{	pad2D.Pad(gfImg, piImgSize, m_gfPadImgN, 0);
		return;
	}
	//---------------------------
	CBufferPool* pBufPool = CBufferPool::GetInstance();
	CStackBuffer* pTmpBuffer = pBufPool->GetBuffer(EBuffer::tmp);
	cufftComplex* gCmpBuf = pTmpBuffer->GetFrame(0, 0);
	pad2D.Pad(gfImg, piImgSize, (float*)gCmpBuf, 0);
	//---------------------------
	Util::CCufft2D* pForFFT = pBufPool->GetForwardFFT(0);
	pForFFT->CreateForwardPlan(piImgSize, false);
	pForFFT->Forward((float*)gCmpBuf, true);
	//---------------------------
	Util::GFourierResize2D fftResize;
	int aiCmpSize[] = {piImgSize[0] / 2 + 1, piImgSize[1]};
	int aiCmpSizeN[] = {m_aiImgSizeN[0] / 2 + 1, m_aiImgSizeN[1]};
	cufftComplex* gCmpImgN = (cufftComplex*)m_gfPadImgN;
	fftResize.DoIt(gCmpBuf, aiCmpSize, gCmpImgN, aiCmpSizeN, 0);
	//---------------------------
	Util::CCufft2D* pInvFFT = pBufPool->GetInverseFFT(0);
	pInvFFT->CreateInversePlan(m_aiImgSizeN, false);
	pInvFFT->Inverse(gCmpImgN);
}

