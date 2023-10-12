#include "CAlignInc.h"
#include "../Correct/CCorrectInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CIterativeAlign::CIterativeAlign(void)
{
	m_iMaxIterations = 7;
	m_iIterations = 0;
	m_fTol = 0.5f;
	m_fBFactor = 150.0f;
	m_bPhaseOnly = false;
	//-------------------
	m_pfErrors = new float[m_iMaxIterations];
	memset(m_pfErrors, 0, sizeof(float) * m_iMaxIterations);
}

CIterativeAlign::~CIterativeAlign(void)
{
	if(m_pfErrors != 0L) delete[] m_pfErrors;
}

void CIterativeAlign::Setup
(	EBuffer eBuffer,
	float fBFactor,
	bool bPhaseOnly
)
{	m_eBuffer = eBuffer;
	m_fBFactor = fBFactor;
	m_bPhaseOnly = bPhaseOnly;
	//------------------------
	CInput* pInput = CInput::GetInstance();	
	m_iMaxIterations = pInput->m_iIterations;
	//---------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	float fBin = fmaxf(pBufferPool->m_afXcfBin[0], 
	   pBufferPool->m_afXcfBin[1]);
	m_fTol = pInput->m_fTolerance / fBin;
	//-----------------------------------		
	if(m_pfErrors != 0L) delete[] m_pfErrors;
	int iSize = m_iMaxIterations * 100;
	m_pfErrors = new float[iSize];
	memset(m_pfErrors, 0, sizeof(float) * iSize);
}

void CIterativeAlign::DoIt
(	DU::CDataPackage* pPackage,
	CStackShift* pStackShift
)
{	nvtxRangePushA("CIterativeAlign::DoIt");
	m_pPackage = pPackage;
	//--------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pStackBuffer = pBufferPool->GetBuffer(m_eBuffer);
	//-------------------------------------------------------------
	int j = (m_eBuffer == EBuffer::pat) ? 1 : 0;
	DU::CFmGroupParam* pFmGroupParam = &m_pPackage->m_pFmGroupParams[j];
	//------------------------------------------------------------------
	pStackShift->Reset();
	pStackShift->m_bConverged = false;
	//--------------------------------
	m_pAlignStacks = new CAlignStack[pBufferPool->m_iNumGpus];
	for(int i=0; i<pBufferPool->m_iNumGpus; i++)
	{	m_pAlignStacks[i].Set1(i);
		m_pAlignStacks[i].Set2(m_eBuffer, pFmGroupParam);
	}
	//-------------------------------------------------------
	m_iIterations = 0;
	float fWeight = 0.3f + 0.1f * pFmGroupParam->m_iBinZ;
	if(fWeight > 0.9f) fWeight = 0.9f;
	//---------------------------------------------------
	CStackShift* pResShift = mAlignStack(pStackShift, 
	   pFmGroupParam->m_iNumGroups);
	pStackShift->SetShift(pResShift);
	if(pResShift != 0L) delete pResShift;
	pStackShift->Smooth(fWeight);
	//---------------------------------------
	if(pFmGroupParam->m_iBinZ >= 2)
	{	CEarlyMotion* pEarlyMotion = new CEarlyMotion;
		pEarlyMotion->Setup(m_eBuffer, m_fBFactor);
		pEarlyMotion->DoIt(m_pPackage, pStackShift);
		delete pEarlyMotion;
	}
	delete[] m_pAlignStacks;
	//---------------------------------------------------
        nvtxRangePop();
	
}

CStackShift* CIterativeAlign::mAlignStack
(	CStackShift* pInitShift,
	int iNumGroups
)
{	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	int aiSumRange[] = {0, pInitShift->m_iNumFrames};
	int k = (m_eBuffer == EBuffer::pat) ? 1 : 0;
	DU::CFmGroupParam* pFmGroupParam = 
	   &(m_pPackage->m_pFmGroupParams[k]);
	//---------------------------------------------
	CStackShift* pTmpShift = pInitShift->GetCopy();
	CStackShift* pGroupShift = new CStackShift;
	CStackShift* pTotalShift = new CStackShift;
	pGroupShift->Setup(iNumGroups);
	pTotalShift->Setup(iNumGroups);
	//------------------------------
	float fBFactor = m_fBFactor;
	CStackShift* pIntShift = 0L;
	float fMaxErr = 0.0f;
	CInterpolateShift aIntpShift;
	//---------------------------
	for(int i=0; i<m_iMaxIterations; i++)
	{	CAlignedSum::DoIt(m_eBuffer, pTmpShift, aiSumRange);
		for(int j=0; j<pBufferPool->m_iNumGpus; j++)
		{	m_pAlignStacks[j].Set3(fBFactor, m_bPhaseOnly);
			m_pAlignStacks[j].DoIt(pTmpShift, pGroupShift);
		}
		fBFactor -= 2;
		if(fBFactor < 20) fBFactor = 20.0f;
		//---------------------------------
		fMaxErr = 0.0f;
		for(int j=0; j<pBufferPool->m_iNumGpus; j++)
		{	m_pAlignStacks[j].WaitStreams();
			float fErr = m_pAlignStacks[j].m_fErr;
			if(fErr > fMaxErr) fMaxErr = fErr;
                }
		//----------------------------------------
		pTotalShift->AddShift(pGroupShift);
		pIntShift = aIntpShift.DoIt(pGroupShift, pFmGroupParam,
		   m_pPackage->m_pFmIntParam, true);
                pTmpShift->AddShift(pIntShift);
		if(pIntShift != 0L) delete pIntShift;
		//-----------------------------------
                m_pfErrors[m_iIterations] = fMaxErr;
                m_iIterations += 1;
		if(fMaxErr < m_fTol && i > 0) break;
        }
	pIntShift = aIntpShift.DoIt(pTotalShift, pFmGroupParam, 
	   m_pPackage->m_pFmIntParam, false);
	pIntShift->RemoveSpikes(false);
	pIntShift->Smooth(0.5f);
	//------------------------------------------------------
	if(pGroupShift != 0L) delete pGroupShift;
	if(pTotalShift != 0L) delete pTotalShift;
	if(pTmpShift != 0L) delete pTmpShift;
	if(fMaxErr < m_fTol) pIntShift->m_bConverged = true;
	return pIntShift;	
}

char* CIterativeAlign::GetErrorLog(void)
{
	int iSize = m_iMaxIterations * 1024;
	char* pcLog = new char[iSize];
	memset(pcLog, 0, sizeof(char) * iSize);
	//-------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	float fXcfBin = fmaxf(pBufferPool->m_afXcfBin[0], 
		pBufferPool->m_afXcfBin[1]);
	char acBuf[80] = {0};
	//-------------------
	for(int i=0; i<m_iIterations; i++)
	{	float fErr = m_pfErrors[i] * fXcfBin;
		sprintf(acBuf, "Iteration %2d  Error: %9.3f\n", i+1, fErr);
		strcat(pcLog, acBuf);
	}
	return pcLog;
}

