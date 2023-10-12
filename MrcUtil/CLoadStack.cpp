#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <memory.h>
#include <string.h>
#include <stdio.h>

using namespace MotionCor2;
using namespace MotionCor2::MrcUtil;

CLoadStack* CLoadStack::m_pInstance = 0L;

CLoadStack* CLoadStack::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CLoadStack;
	return m_pInstance;
}

void CLoadStack::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CLoadStack::CLoadStack(void)
{
	m_pMrcStack = 0L;
}

CLoadStack::~CLoadStack(void)
{
	if(m_pMrcStack != 0L) delete m_pMrcStack;
}

bool CLoadStack::OpenFile(char* pcMrcFile)
{	
	if(pcMrcFile != 0L) strcpy(m_acMrcFile, pcMrcFile);
	else strcpy(m_acMrcFile, "");
	m_bSerialFiles = false;
	//---------------------
	CAnalyzeMrc aAnalyzeMrc;
	if(!aAnalyzeMrc.IsMrc(m_acMrcFile)) return false;
	//-----------------------------------------------
	int aiImgSize[3] = {0};
	aAnalyzeMrc.GetMrcSize(m_acMrcFile, aiImgSize, &m_iNumStacks);
	if(!aAnalyzeMrc.IsTomoMrc(m_acMrcFile)) return true;
	//--------------------------------------------------
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iStackZ > 1) 
	{	m_iNumStacks = aiImgSize[2] / pInput->m_iStackZ;
	}
	return true;
}

bool CLoadStack::Load(int iStack)
{
	if(m_pMrcStack != 0L) delete m_pMrcStack;
	m_pMrcStack = new DataUtil::CMrcStack;
	m_pMrcStack->m_iStack = iStack;
	//-----------------------------
	bool bLoad = true;
	if(m_bSerialFiles) bLoad = mLoadSerial();
	else bLoad = mLoadSingle();
	//-------------------------
	int iLeft = m_iNumStacks - 1 - iStack;
	if(bLoad) printf("Stack %04d has been loaded, ", iStack+1);
	else printf("Stack %04d has been skipped, ", iStack+1);
	printf("%d stacks left.\n\n", iLeft);
	return bLoad;
}

DataUtil::CMrcStack* CLoadStack::GetStack(bool bClean)
{
	DataUtil::CMrcStack* pMrcStack = m_pMrcStack;
	if(bClean) m_pMrcStack = 0L;
	return pMrcStack;
}

bool CLoadStack::mLoadSingle(void)
{	
	Mrc::CLoadMrc aLoadMrc;
	if(!aLoadMrc.OpenFile(m_acMrcFile)) 
	{	fprintf
		(  stderr, "CLoadStack::mLoadSingle: "
		   "cannot open stack file, skip,\n"
		   "   %s\n\n", m_acMrcFile
		);
		return false;
	}
	//-------------------
	CInput* pInput = CInput::GetInstance();
	int iMode = aLoadMrc.m_pLoadMain->GetMode();
	m_aiStkSize[0] = aLoadMrc.m_pLoadMain->GetSizeX();
	m_aiStkSize[1] = aLoadMrc.m_pLoadMain->GetSizeY();
	m_aiStkSize[2] = aLoadMrc.m_pLoadMain->GetSizeZ();
	float fPixelSize = aLoadMrc.GetPixelSize();
	//-----------------------------------------
	int iStackZ = aLoadMrc.m_pLoadMain->GetStackZ();
	if(pInput->m_iStackZ > 1) iStackZ = pInput->m_iStackZ;
	if(iStackZ > 1) m_aiStkSize[2] = iStackZ;
	mPrintStackInfo(m_aiStkSize, iMode);
	m_pMrcStack->Create(iMode, m_aiStkSize);
	//--------------------------------------
	bool bZero = true;
	int iStartFrame = m_pMrcStack->m_iStack * m_aiStkSize[2];
	//-------------------------------------------------------
	for(int i=0; i<m_aiStkSize[2]; i++)
	{	int j = iStartFrame + i;
		void* pvFrame = m_pMrcStack->GetFrame(i);
		aLoadMrc.m_pLoadImg->DoIt(j, pvFrame);
		//float fMean = mCalcMean(pcFrame);
		//printf("...... Stack %003d  frame %003d  mean:  %f\n",
		//   iStack + 1, i + 1 + m_iThrow, fMean);
	}
	//------------------------------------------------
	aLoadMrc.m_pLoadExt->DoIt(iStartFrame);
	for(int i=0; i<m_pMrcStack->m_iNumFloats; i++)
	{	float fVal = aLoadMrc.m_pLoadExt->GetNthFloat(i);
		m_pMrcStack->m_afExt[i] = fVal;
	}
	m_pMrcStack->m_afExt[11] = fPixelSize;
	return true;
}

bool CLoadStack::mLoadSerial(void)
{	
	Mrc::CLoadMrc aLoadMrc;
	char acSerialMrc[512];
	int iStack = m_pMrcStack->m_iStack;
	sprintf(acSerialMrc, "%s%d", m_acMrcFile, iStack);
	if(!aLoadMrc.OpenFile(acSerialMrc))
	{	fprintf(stderr, "CLoadStack::mLoadSerial: "
			"cannot open stack file, skip,\n"
			"   %s\n\n", acSerialMrc);
		return false;
	}
	//-------------------
	int iMode = aLoadMrc.m_pLoadMain->GetMode();
	m_aiStkSize[0] = aLoadMrc.m_pLoadMain->GetSizeX();
	m_aiStkSize[1] = aLoadMrc.m_pLoadMain->GetSizeY();
	m_aiStkSize[2] = aLoadMrc.m_pLoadMain->GetSizeZ();
	mPrintStackInfo(m_aiStkSize, iMode);
	m_pMrcStack->Create(iMode, m_aiStkSize);
	//--------------------------------------
	char acBuf[128];
	char* pcLog = new char[256 * m_aiStkSize[2]];
	strcpy(pcLog, "");
	const char* pcForm1 = "...... Stack %03d frame %03d  mean: %f\n";
	const char* pcForm2 = "...... Ext header (%02d): %8.2f\n"; 
	bool bClean = true;
	//-----------------
	for(int i=0; i<m_aiStkSize[2]; i++)
	{	void* pvFrame = m_pMrcStack->GetFrame(i);
        	aLoadMrc.m_pLoadImg->DoIt(i, pvFrame);
		float fMean = mCalcMean((char*)pvFrame);
		sprintf(acBuf, pcForm1, iStack + 1, i + 1, fMean);
		strcat(pcLog, acBuf);
	}
	//------------------------
	aLoadMrc.OpenFile(m_acMrcFile);
	aLoadMrc.m_pLoadExt->DoIt(0);
	for(int i=0; i<m_pMrcStack->m_iNumFloats; i++)
	{	float fVal = aLoadMrc.m_pLoadExt->GetNthFloat(i);
		m_pMrcStack->m_afExt[i] = fVal;
		sprintf(acBuf, pcForm2, i+1, fVal);
		strcat(pcLog, acBuf);
	}
	float fPixelSize = aLoadMrc.GetPixelSize();
	m_pMrcStack->m_afExt[11] = fPixelSize;
	printf("%s\n\n", pcLog);
	if(pcLog != 0L) delete[] pcLog;
	return true;
}

float CLoadStack::mCalcMean(char* pcFrame)
{
	int iMode = m_pMrcStack->m_iMode;
	int iPixels = m_pMrcStack->m_aiStkSize[0] 
		* m_pMrcStack->m_aiStkSize[1];
	double dMean = 0.0;
	//-----------------
	if(iMode == Mrc::eMrc4Bits)
	{	unsigned char* pucBuf = new unsigned char[iPixels];
		Mrc::C4BitImage::Unpack(pcFrame, pucBuf,
			m_pMrcStack->m_aiStkSize);
		for(int i=0; i<iPixels; i++) dMean += pucBuf[i];
		delete[] pucBuf;
		dMean = dMean / iPixels;
	}
	else if(iMode == Mrc::eMrcUChar || iMode == Mrc::eMrcUCharEM)
	{	unsigned char* pucFrame = (unsigned char*)pcFrame;
		for(int i=0; i<iPixels; i++) dMean += pucFrame[i];
		dMean = dMean / iPixels;
	}
	else if(iMode == Mrc::eMrcFloat)
	{	float* pfFrame = (float*)pcFrame;
		for(int i=0; i<iPixels; i++) dMean += pfFrame[i];
		dMean = dMean / iPixels;
	}
	return (float)dMean;	
}

void CLoadStack::mPrintStackInfo(int* piStkSize, int iMode)
{
	printf("Stack size: %d  %d  %d\n", piStkSize[0],
                piStkSize[1], piStkSize[2]);
        printf("Stack mode: %d\n", iMode);
}

