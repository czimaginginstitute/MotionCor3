#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <Util/Util_Time.h>
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CMeasurePatches::CMeasurePatches(void)
{
}

CMeasurePatches::~CMeasurePatches(void)
{
}

void CMeasurePatches::DoIt
(	DU::CDataPackage* pPackage,
	CPatchShifts* pPatchShifts
)
{	m_pPackage = pPackage;
	m_pPatchShifts = pPatchShifts;
	//------------------------------
	CInput* pInput = CInput::GetInstance();
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	//----------------------------------------------------
	float fBin = fmaxf(pBufferPool->m_afXcfBin[0], 
	   pBufferPool->m_afXcfBin[1]);
	m_fTol = pInput->m_fTolerance / fBin;
	m_bPhaseOnly = (pInput->m_iPhaseOnly == 0) ? false : true;
	//--------------------------------------------------------
	int iNumPatches = pInput->m_aiNumPatches[0] 
	   * pInput->m_aiNumPatches[1];
	for(int i=0; i<iNumPatches; i++)
	{	CExtractPatch::DoIt(i);
		mCalcPatchShift(i);
	}
	m_pPatchShifts->MakeRelative();
	m_pPatchShifts->DetectBads();
}

void CMeasurePatches::mCalcPatchShift(int iPatch)
{
	CInput* pInput = CInput::GetInstance();	
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	CStackBuffer* pPatBuffer = pBufferPool->GetBuffer(EBuffer::pat);
	//--------------------------------------------------------------
	bool bForward = true, bNorm = true;
	CTransformStack::DoIt(EBuffer::pat, bForward, !bNorm);
	//----------------------------------------------------
	CStackShift* pPatShift = new CStackShift;
	pPatShift->Setup(pPatBuffer->m_iNumFrames);
	//-----------------------------------------
	CIterativeAlign iterAlign;
	iterAlign.Setup(EBuffer::pat, 
	   pInput->m_afBFactor[1], m_bPhaseOnly);
	iterAlign.DoIt(m_pPackage, pPatShift);
	//------------------------------------
	int iNumPatches = pInput->m_aiNumPatches[0] 
	   * pInput->m_aiNumPatches[1];
	char* pcErrLog = iterAlign.GetErrorLog();
	int iLeft = iNumPatches - 1 - iPatch;
	printf("Align patch %d  %d left\n%s\n", iPatch+1, iLeft, pcErrLog);
	if(pcErrLog != 0L) delete[] pcErrLog;
	//-----------------------------------------------------------------
	int aiCenter[2] = {0};
	CPatchCenters* pPatchCenters = CPatchCenters::GetInstance();
	pPatchCenters->GetCenter(iPatch, aiCenter);
	float fCentX = aiCenter[0] * pBufferPool->m_afXcfBin[0];
	float fCentY = aiCenter[1] * pBufferPool->m_afXcfBin[1];
	pPatShift->SetCenter(fCentX, fCentY);
	//-----------------------------------
	pPatShift->Multiply(pBufferPool->m_afXcfBin[0],
	   pBufferPool->m_afXcfBin[1]);
	m_pPatchShifts->SetRawShift(pPatShift, iPatch);
	if(pPatShift != 0L) delete pPatShift;
}

