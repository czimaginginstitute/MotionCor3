#include "CAlignInc.h"
#include "../Correct/CCorrectInc.h"
#include "../Util/CUtilInc.h"
#include <Util/Util_Time.h>
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CPatchAlign::CPatchAlign(void)
{
	m_pPatchShifts = 0L;
}

CPatchAlign::~CPatchAlign(void)
{
	if(m_pPatchShifts != 0L) delete m_pPatchShifts;
}

void CPatchAlign::DoIt(DU::CDataPackage* pPackage)
{
	//----------------	
	if(m_pPatchShifts != 0L)
	{	delete m_pPatchShifts;
		m_pPatchShifts = 0L;
	}
	CFullAlign::Align(pPackage);
	mCorrectFullShift();
	//------------------
	Util_Time aTimer;
	aTimer.Measure();
	mCalcPatchShifts();
	float fSeconds = aTimer.GetElapsedSeconds();
	printf("Patch alignment time: %.2f(sec)\n\n", fSeconds);
	//---------------------------
	Correct::GCorrectPatchShift::DoIt(m_pPatchShifts, pPackage); 
	//---------------------------
	CSaveAlign* pSaveAlign = CSaveAlign::GetInstance();
	pSaveAlign->DoLocal(m_pPatchShifts);
}

void CPatchAlign::mCorrectFullShift(void)
{
	nvtxRangePushA("CPatchAlign::mCorrectFullShift");
	CAlignParam* pAlignParam = CAlignParam::GetInstance();
	int iRefFrame = pAlignParam->GetFrameRef(m_pFullShift->m_iNumFrames);
	m_pFullShift->MakeRelative(iRefFrame);
	//---------------------------
	CInput* pInput = CInput::GetInstance();
	bool bCorrInterp = (pInput->m_iCorrInterp == 0) ? false : true;
	bool bMotionDecon = (pInput->m_iInFmMotion == 0) ? false : true; 
	bool bGenReal = true;
	//---------------------------
	Util_Time utilTime; utilTime.Measure();
	Correct::CGenRealStack::DoIt(EBuffer::frm,
	   bCorrInterp, bMotionDecon, !bGenReal, m_pFullShift);
	printf("Global shifts are corrected: %f sec\n",
		utilTime.GetElapsedSeconds());
	nvtxRangePop();
}

void CPatchAlign::mCalcPatchShifts(void)
{
	nvtxRangePushA("CPatchAlign::mCalcPatchShifts");
	CLoadAlign* pLoadAlign = CLoadAlign::GetInstance();
	CAlignParam* pAlignParam = CAlignParam::GetInstance();
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	//----------------------------------------------------
	if(pLoadAlign->IsLoaded())
	{	bool bClean = true;
		m_pPatchShifts = pLoadAlign->GetPatchShifts(bClean);
		m_pPatchShifts->SetFullShift(m_pFullShift);
	}
	else
	{	m_pPatchShifts = new CPatchShifts;
		int* piStkSize = pBufferPool->m_aiStkSize;
		m_pPatchShifts->Setup(pAlignParam->GetNumPatches(), piStkSize);
		m_pPatchShifts->SetFullShift(m_pFullShift);
		//-------------------------------------
		printf("Start to align patches.\n");
		CMeasurePatches meaPatches;
		meaPatches.DoIt(m_pPackage, m_pPatchShifts);
     	}
	m_pFullShift = 0L;
	//----------------
	nvtxRangePop();
}

void CPatchAlign::LogShift(char* pcLogFile)
{
	m_pPatchShifts->LogFullShifts(pcLogFile);
	m_pPatchShifts->LogPatchShifts(pcLogFile);
	m_pPatchShifts->LogFrameShifts(pcLogFile);
}

