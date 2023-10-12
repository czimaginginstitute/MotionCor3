#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../DataUtil/CDataUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;
namespace DU = MotionCor2::DataUtil;

CAlignBase::CAlignBase(void)
{
	m_pFullShift = 0L;
}

CAlignBase::~CAlignBase(void)
{
	this->Clean();
}

void CAlignBase::Clean(void)
{
	if(m_pFullShift == 0L) return;
	delete m_pFullShift;
	m_pFullShift = 0L;
}

void CAlignBase::DoIt(DU::CDataPackage* pPackage)
{
	nvtxRangePushA ("CAlignBase::DoIt");
	//----------------------------------
	m_pPackage = pPackage;
	CInput* pInput = CInput::GetInstance();
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	//--------------------------------------------------------------
	Util::GFourierResize2D::GetBinnedCmpSize(pFrmBuffer->m_aiCmpSize,
	   pInput->m_fFourierBin, m_aiCmpSize);
	//-------------------------------------
	m_aiImgSize[0] = (m_aiCmpSize[0] - 1) * 2;
	m_aiImgSize[1] = m_aiCmpSize[1];
	m_aiPadSize[0] = m_aiCmpSize[0] * 2;
	m_aiPadSize[1] = m_aiCmpSize[1];
	//------------------------------
	mCreateAlnSums();
	mCreateAlnStack();
	nvtxRangePop();
}

void CAlignBase::LogShift(char* pcLogFile)
{
}

void CAlignBase::mCreateAlnSums(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	int iNumSums = pBufferPool->m_iNumSums;
	int aiStkSize[] = {m_aiImgSize[0], m_aiImgSize[1], iNumSums};
	//-----------------------------------------------------------
	bool bCreatePkg = true;
	if(m_pPackage->m_pAlnSums != 0L)
	{	int* piOldSize = m_pPackage->m_pAlnSums->m_aiStkSize;
		if(piOldSize[0] == m_aiImgSize[0] &&
		   piOldSize[1] == m_aiImgSize[1] &&
		   piOldSize[2] == iNumSums) return;
	}
	//------------------------------------------
	if(m_pPackage->m_pAlnSums != 0L)
	{	delete m_pPackage->m_pAlnSums;
		m_pPackage->m_pAlnSums = 0L;
	}
	DU::CAlnSums* pAlnSums = new DU::CAlnSums;
	pAlnSums->Create(Mrc::eMrcFloat, aiStkSize);
	m_pPackage->m_pAlnSums = pAlnSums;
	//--------------------------------	
	DU::CFmIntegrateParam* pFmIntParam = m_pPackage->m_pFmIntParam;
	pAlnSums->Setup(pFmIntParam->DoseWeight(),
	   pFmIntParam->DWSelectedSum());
	//-------------------------------
	CInput* pInput = CInput::GetInstance();
	pAlnSums->m_fPixSize = pInput->GetFinalPixelSize();
}

void CAlignBase::mCreateAlnStack(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_aiOutStack[0] == 0) return;
	//--------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	int iNumFrames = pFrmBuffer->m_iNumFrames;
	int aiStkSize[] = {m_aiImgSize[0], m_aiImgSize[1], iNumFrames};
	//-------------------------------------------------------------
	DataUtil::CMrcStack* pAlnStack = new DataUtil::CMrcStack;
	pAlnStack->Create(Mrc::eMrcFloat, aiStkSize);
	pAlnStack->m_fPixSize = pInput->GetFinalPixelSize();
	m_pPackage->m_pAlnStack = pAlnStack;

}
