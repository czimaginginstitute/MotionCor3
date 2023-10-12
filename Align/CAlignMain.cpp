#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CAlignMain::CAlignMain(void)
{
	m_pAlnSums = 0L;
	m_pAlnStack = 0L;
}

CAlignMain::~CAlignMain(void)
{
	mClean();
}

DataUtil::CAlnSums* CAlignMain::GetAlnSums(bool bClean)
{
	DataUtil::CAlnSums* pAlnSums = m_pAlnSums;
	if(bClean) m_pAlnSums = 0L;
	return pAlnSums;
}

DataUtil::CMrcStack* CAlignMain::GetAlnStack(bool bClean)
{
	DataUtil::CMrcStack* pAlnStack = m_pAlnStack;
	if(bClean) m_pAlnStack = 0L;
	return pAlnStack;
}

void CAlignMain::DoIt(DU::CDataPackage* pPackage)
{
	nvtxRangePush ("CAlignMain::DoIt");
	mClean();
	m_pPackage = pPackage;
	//--------------------
	CAlignBase* pAlignBase = 0L;
	CAlignParam* pAlignParam = CAlignParam::GetInstance();
	//----------------------------------------------------
	nvtxRangePushA("align select");
	if(pAlignParam->SimpleSum())
	{	printf("Simple sum\n");
		pAlignBase = new CSimpleSum;
	}
	else if(pAlignParam->PatchAlign()) 
	{	printf("Patch based alignment\n");
		pAlignBase = new CPatchAlign;
	}
	else
	{	printf("Full frame alignment\n");
		pAlignBase = new CFullAlign;
	}
	nvtxRangePop();
	pAlignBase->DoIt(pPackage);
	//-------------------------
	CZbinStack aZbinStack;
	aZbinStack.DoIt(pPackage);
	//------------------------
	char* pcLogFile = mCreateLogFile();
	if(pcLogFile != 0L)
	{	pAlignBase->LogShift(pcLogFile);
		delete[] pcLogFile;
	}
	//--------------------------------------------
	if(pAlignBase != 0L) delete pAlignBase;
	nvtxRangePop();
}

char* CAlignMain::mCreateLogFile(void)
{
	CInput* pInput = CInput::GetInstance();
	if(0 == strlen(pInput->m_acLogDir)) return 0L;
	//--------------------------------------------
	char* pcInFile = strrchr(m_pPackage->m_pcInFileName, '/');
	if(pcInFile != 0L) pcInFile += 1;
	else pcInFile = m_pPackage->m_pcInFileName;
	//------------------------------------------------------
	char* pcLogFile = new char[256];
	strcpy(pcLogFile, pInput->m_acLogDir);
	strcat(pcLogFile, pcInFile);
	//------------------------------------
	char* pcFileExt = strcasestr(pcLogFile, ".mrc");
	if(pcFileExt != 0L) 
	{	strcpy(pcFileExt, "");
		return pcLogFile;
	}
	pcFileExt = strcasestr(pcLogFile, ".tif");
	if(pcFileExt != 0L)
	{	strcpy(pcFileExt, "");
		return pcLogFile;
	}
	pcFileExt = strcasestr(pcLogFile, ".eer");
	if(pcFileExt != 0L)
	{	strcpy(pcFileExt, "");
		return pcLogFile;
	}
	//------------------------------------------------
	delete pcLogFile; return 0L;
}

void CAlignMain::mClean(void)
{
	if(m_pAlnStack != 0L) delete m_pAlnStack;
	m_pAlnStack = 0L;
}
