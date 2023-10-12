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

CZbinStack::CZbinStack(void)
{
}

CZbinStack::~CZbinStack(void)
{
}

//-----------------------------------------------------------------
// Need to revisit !!!
//-----------------------------------------------------------------
void CZbinStack::DoIt(DU::CDataPackage* pPackage)
{
	DU::CMrcStack* pAlnStack = pPackage->m_pAlnStack;
	if(pAlnStack == 0L) return;
	//-------------------------
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_aiOutStack[0] <= 0) return;
	if(pInput->m_aiOutStack[1] <= 1) return;
	//--------------------------------------
	m_pMrcStack = pAlnStack;
	DU::CFmGroupParam aFmGroupParam;
	aFmGroupParam.Setup(pInput->m_aiOutStack[1], pPackage->m_pFmIntParam);
	//--------------------------------------------------------------------
	int iNumGroups = aFmGroupParam.m_iNumGroups;
	for(int i=0; i<iNumGroups; i++)
	{	int iGroupStart = aFmGroupParam.GetGroupStart(i);
		int iGroupSize = aFmGroupParam.GetGroupSize(i);
		mGroup(i, iGroupStart, iGroupSize);
	}
	//-----------------------------------------------------
	// delete remaining frames in the aligned stack.
	//-----------------------------------------------------
	for(int i=iNumGroups; i<pAlnStack->m_aiStkSize[2]; i++)
	{	pAlnStack->DeleteFrame(i);
	}
	pAlnStack->m_aiStkSize[2] = iNumGroups;
	m_pMrcStack = 0L;
}

void CZbinStack::mGroup(int iGroup, int iGroupStart, int iGroupSize)
{
	float* pfBinnedFm = (float*)m_pMrcStack->GetFrame(iGroup);
	float* pfFm = (float*)m_pMrcStack->GetFrame(iGroupStart);
	if(iGroup != iGroupStart)
	{	memcpy(pfBinnedFm, pfFm, m_pMrcStack->m_tFmBytes);
	}
	int iPixels = m_pMrcStack->GetPixels();
	for(int i=1; i<iGroupSize; i++)
	{	pfFm = (float*)m_pMrcStack->GetFrame(i+iGroupStart);
		for(int j=0; j<iPixels; j++)
		{	pfBinnedFm[j] += pfFm[j];
		}
	}
}

