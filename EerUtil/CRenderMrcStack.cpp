#include "CEerUtilInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2;
namespace DU = MotionCor2::DataUtil;
using namespace MotionCor2::EerUtil;

CRenderMrcStack::CRenderMrcStack(void)
{
}

CRenderMrcStack::~CRenderMrcStack(void)
{
}

void CRenderMrcStack::DoIt
(	CLoadEerHeader* pLoadEerHeader,
	CLoadEerFrames* pLoadEerFrames,
	void* pvDataPackage
)
{	m_pLoadEerHeader = pLoadEerHeader;
	m_pLoadEerFrames = pLoadEerFrames;
	m_aDecodeEerFrame.Setup(m_pLoadEerHeader->m_aiCamSize, 
	   m_pLoadEerHeader->m_iEerSampling);
	//-----------------------------------
	DU::CDataPackage* pPackage = (DU::CDataPackage*)pvDataPackage;
	if(pPackage->m_pFmIntParam->NeedIntegrate()) 
	{	mRenderInt(pvDataPackage);
	}
	else mRender(pvDataPackage);
}

void CRenderMrcStack::mRender(void* pvDataPackage)
{
	DU::CDataPackage* pPackage = (DU::CDataPackage*)pvDataPackage;
	DU::CMrcStack* pRawStack = pPackage->m_pRawStack;
	//-----------------------------------------------
	unsigned char* pucMrcFrm = 0L;
	for(int i=0; i<pRawStack->m_aiStkSize[2]; i++)
	{	pucMrcFrm = (unsigned char*)pRawStack->GetFrame(i);
		memset(pucMrcFrm, 0, pRawStack->m_tFmBytes);
		int iEerFrm = pPackage->m_pFmIntParam->GetIntFmStart(i);
		mDecodeFrame(iEerFrm, pucMrcFrm);
	}
}

void CRenderMrcStack::mRenderInt(void* pvDataPackage)
{
	DU::CDataPackage* pPackage = (DU::CDataPackage*)pvDataPackage;
	DU::CMrcStack* pRawStack = pPackage->m_pRawStack;
	DU::CFmIntegrateParam* pFmIntParam = pPackage->m_pFmIntParam;
	//-----------------------------------------------------------------
	for(int i=0; i<pRawStack->m_aiStkSize[2]; i++)
	{	int iFmSize = pFmIntParam->GetIntFmSize(i);
		if(iFmSize == 1) 
		{	void* pvMrcFrm = pRawStack->GetFrame(i);
			memset(pvMrcFrm, 0, pRawStack->m_tFmBytes);
			int iFmStart = pFmIntParam->GetIntFmStart(i);
			mDecodeFrame(iFmStart, (unsigned char*)pvMrcFrm);
		}
		else mRenderFrame(i, pvDataPackage);
	}
}

void CRenderMrcStack::mRenderFrame(int iIntFrm, void* pvDataPackage)
{
	DU::CDataPackage* pPackage = (DU::CDataPackage*)pvDataPackage;
	DU::CMrcStack* pRawStack = pPackage->m_pRawStack;
	DU::CFmIntegrateParam* pFmIntParam = pPackage->m_pFmIntParam;
	//-----------------------------------------------------------
	int iIntFmStart = pFmIntParam->GetIntFmStart(iIntFrm);
	int iIntFmSize = pFmIntParam->GetIntFmSize(iIntFrm);
	//--------------------------------------------------
	unsigned char* pucFrm = (unsigned char*)pRawStack->GetFrame(iIntFrm);
	memset(pucFrm, 0, pRawStack->m_tFmBytes);
	for(int i=0; i<iIntFmSize; i++)
	{	int iEerFrame = iIntFmStart + i;
		mDecodeFrame(iEerFrame, pucFrm);
	}
}	

void CRenderMrcStack::mDecodeFrame
(	int iEerFrame,
	unsigned char* pucDecodedFrm
)
{	int iEerFmBytes = m_pLoadEerFrames->GetEerFrameSize(iEerFrame);
	int iEerBits = m_pLoadEerHeader->m_iNumBits;
	unsigned char* pEerFrm = m_pLoadEerFrames->GetEerFrame(iEerFrame);
	if(iEerBits == 7)
	{	m_aDecodeEerFrame.Do7Bits(pEerFrm, 
		   iEerFmBytes, pucDecodedFrm);
	}
	else
	{	m_aDecodeEerFrame.Do8Bits(pEerFrm, 
		   iEerFmBytes, pucDecodedFrm);
	}
}

