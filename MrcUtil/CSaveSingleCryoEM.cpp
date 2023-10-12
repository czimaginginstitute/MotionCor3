#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <Util/Util_Time.h>

using namespace MotionCor2;
using namespace MotionCor2::MrcUtil;

CSaveSingleCryoEM* CSaveSingleCryoEM::m_pInstance = 0L;

CSaveSingleCryoEM* CSaveSingleCryoEM::GetInstance(void)
{
	if(m_pInstance == 0L) m_pInstance = new CSaveSingleCryoEM;
	return m_pInstance;
}

void CSaveSingleCryoEM::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CSaveSingleCryoEM::CSaveSingleCryoEM(void)
{
}

CSaveSingleCryoEM::~CSaveSingleCryoEM(void)
{
}

void CSaveSingleCryoEM::OpenFile(char* pcMrcFile)
{
	strcpy(m_acMrcFile, pcMrcFile);
	strcpy(m_acMrcFileStk, pcMrcFile);
	//--------------------------------
	char* pcMrcExt = strstr(m_acMrcFileStk, ".mrc");
	if(pcMrcExt == 0L) strcat(m_acMrcFileStk, "_Stk.mrc");
	else strcpy(pcMrcExt, "_Stk.mrc");
}

void CSaveSingleCryoEM::DoIt
(	DataUtil::CAlnSums* pAlnSums, 
	DataUtil::CMrcStack* pAlnStack,
	float fPixelSize
)
{	Util_Time aTimer;
	aTimer.Measure();
	//---------------
	CInput* pInput = CInput::GetInstance();
	cudaSetDevice(pInput->m_piGpuIds[0]);
	cudaMalloc(&m_gfImg, pAlnSums->m_tFmBytes);
	//-----------------------------------------
	Util::GFindMinMax2D* pGFindMinMax = new Util::GFindMinMax2D;
	Util::GCalcMoment2D* pGCalcMoment = new Util::GCalcMoment2D;
	pGFindMinMax->SetSize(pAlnSums->m_aiStkSize, false);
	pGCalcMoment->SetSize(pAlnSums->m_aiStkSize, false);
	m_pvGFindMinMax2D = pGFindMinMax;
	m_pvGCalcMoment2D = pGCalcMoment;
	//-------------------------------
	printf("Saving motion corrected images ......\n");
	m_fPixelSize = fPixelSize;
	char acMrcFile[512], acFileExt[32];
	for(int i=0; i<pAlnSums->m_aiStkSize[2]; i++)
	{	float* pfAlnSum = (float*)pAlnSums->GetFrame(i);
		pAlnSums->GetFileExt(i, acFileExt);
		strcpy(acMrcFile, m_acMrcFile);
		char* pcExt = strstr(acMrcFile, ".mrc");
		if(pcExt == 0L) strcat(acMrcFile, acFileExt);
		else strcpy(pcExt, acFileExt);
		mSave(acMrcFile, pfAlnSum, pAlnSums->m_aiStkSize);
	}
	if(pAlnSums != 0L) delete pAlnSums;
	printf("Saving motion corrected images done.\n");
	//-----------------------------------------------
	mSave(m_acMrcFileStk, pAlnStack);
	if(pAlnStack != 0L) delete pAlnStack;
	//-----------------------------------
	if(m_gfImg != 0L) cudaFree(m_gfImg);
	m_gfImg = 0L;
	delete pGFindMinMax; m_pvGFindMinMax2D = 0L;
	delete pGCalcMoment; m_pvGCalcMoment2D = 0L;
	printf("Save image sums: %.2f sec\n\n", aTimer.GetElapsedSeconds());
}

void CSaveSingleCryoEM::mSave
(	char* pcMrcFile, 
	float* pfImg, 
	int* piImgSize
)
{	if(pfImg == 0L) return;
	//---------------------
	Mrc::CSaveMrc aSaveMrc;
	if(!aSaveMrc.OpenFile(pcMrcFile)) return;
	//---------------------------------------	
	aSaveMrc.SetMode(Mrc::eMrcFloat);
	aSaveMrc.SetImgSize(piImgSize, 1, 1, m_fPixelSize);
	aSaveMrc.SetExtHeader(0, 0, 0);
	//-----------------------------
	size_t tBytes = sizeof(float) * piImgSize[0] * piImgSize[1];
	cudaMemcpy(m_gfImg, pfImg, tBytes, cudaMemcpyDefault);
	//----------------------------------------------------
	Util::GFindMinMax2D* pGFindMinMax = (Util::GFindMinMax2D*)
	   m_pvGFindMinMax2D;
	Util::GCalcMoment2D* pGCalcMoment = (Util::GCalcMoment2D*)
	   m_pvGCalcMoment2D;
	float fMin = pGFindMinMax->DoMin(m_gfImg, true);
	float fMax = pGFindMinMax->DoMax(m_gfImg, true);
	float fMean = pGCalcMoment->DoIt(m_gfImg, 1, true);
	//-------------------------------------------------
	aSaveMrc.SaveMinMaxMean(fMin, fMax, fMean);
	aSaveMrc.DoIt(0, pfImg);
}

void CSaveSingleCryoEM::mSave
(	char* pcMrcFile,
	DataUtil::CMrcStack* pMrcStack
)
{	if(pMrcStack == 0L || pMrcStack->m_aiStkSize[2] <= 0) return;
	//-----------------------------------------------------------
	Mrc::CSaveMrc aSaveMrc;
	remove(pcMrcFile);
	if(!aSaveMrc.OpenFile(pcMrcFile)) return;
	//---------------------------------------
	printf("Saving motion corrected stack ......\n");
	aSaveMrc.SetMode(Mrc::eMrcFloat);
	aSaveMrc.SetImgSize(pMrcStack->m_aiStkSize, 
	   pMrcStack->m_aiStkSize[2], 1, m_fPixelSize);
	aSaveMrc.SetExtHeader(0, 32, 0);
	//------------------------------
	Util::GFindMinMax2D *pGFindMinMax = 
	   (Util::GFindMinMax2D*)m_pvGFindMinMax2D;
	//-----------------------------------------
	Util::GCalcMoment2D* pGCalcMoment = (Util::GCalcMoment2D*)
	   m_pvGCalcMoment2D;
	//-------------------
	float fMin = (float)1e20, fMax = (float)-1e20;
	double dMean = 0.0;
	//-----------------
	for(int i=0; i<pMrcStack->m_aiStkSize[2]; i++)
	{	void* pvFrame = pMrcStack->GetFrame(i);
		cudaMemcpy(m_gfImg, pvFrame, pMrcStack->m_tFmBytes,
		   cudaMemcpyDefault);
		float ffMin = pGFindMinMax->DoMin(m_gfImg, true);
		float ffMax = pGFindMinMax->DoMax(m_gfImg, true);
		float ffMean = pGCalcMoment->DoIt(m_gfImg, 1, true);
		//--------------------------------------------------
		if(fMin > ffMin) fMin = ffMin;
		if(fMax < ffMax) fMax = ffMax;
		dMean += ffMean;
	}
	dMean = dMean / pMrcStack->m_aiStkSize[2];
	aSaveMrc.SaveMinMaxMean(fMin, fMax, (float)dMean);
	//------------------------------------------------
	int iLast = pMrcStack->m_aiStkSize[2] - 1;
	for(int i=0; i<pMrcStack->m_aiStkSize[2]; i++)
	{	void* pvFrm = pMrcStack->GetFrame(i);
		aSaveMrc.m_pSaveExt->SetPixelSize(i, m_fPixelSize);
		aSaveMrc.DoIt(i, pvFrm);	
		if(i % 10 != 0 && i != iLast) continue;
		printf("....... Frame %4d saved, %4d left\n", i+1, iLast-i);
	}
	aSaveMrc.CloseFile();
	printf("Saving motion corrected stack done.\n");
}
