#include "CMainInc.h"
#include "DataUtil/CDataUtilInc.h"
#include "FindCtf/CFindCtfInc.h"
#include <stdio.h>
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2;
namespace DU = MotionCor2::DataUtil;
namespace MU = MotionCor2::MrcUtil;

void sGenOutputName
(	const char* pcSerial, 
	const char* pcSuffix, 
	const char* pcFileExt,
	char* pcOutName)

{
	CInput* pInput = CInput::GetInstance();
	strcpy(pcOutName, pInput->m_acOutMrcFile);
	//-----------------
	char* pcMrcExt = strcasestr(pcOutName, ".mrc");
	if(pcMrcExt != 0L) pcMrcExt[0] = '\0';
	//-----------------
	char* pcStExt = strcasestr(pcOutName, ".st");
	if(pcStExt != 0L) pcStExt[0] = '\0';
	//-----------------
	if(pcSerial != 0L) strcat(pcOutName, pcSerial);
	if(pcSuffix != 0L) strcat(pcOutName, pcSuffix);
	if(pcFileExt != 0L) strcat(pcOutName, pcFileExt);
}

CSaveSerialCryoEM* CSaveSerialCryoEM::m_pInstance = 0L;

CSaveSerialCryoEM* CSaveSerialCryoEM::GetInstance(void)
{
	if(m_pInstance == 0L) m_pInstance = new CSaveSerialCryoEM;
	return m_pInstance;
}

void CSaveSerialCryoEM::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CSaveSerialCryoEM::CSaveSerialCryoEM(void)
{
	m_gfImg = 0L;
}

CSaveSerialCryoEM::~CSaveSerialCryoEM(void)
{
}

//-------------------------------------------------------------------
// 1. When motion-corrected stack is ready for saving, we need to 
//    wait until the previous stack has been saved.
// 2. This is because buffering two or more stacks takes too much
//    CPU memory.
//-------------------------------------------------------------------
void CSaveSerialCryoEM::AsyncSave(DU::CDataPackage* pPackage)
{	
	pthread_mutex_lock(&m_aMutex);
	m_aSaveQueue.push(pPackage);
	pthread_mutex_unlock(&m_aMutex);
	if(!IsAlive()) this->Start();
}

void CSaveSerialCryoEM::ThreadMain(void)
{
	CInput* pInput = CInput::GetInstance();
	cudaSetDevice(pInput->m_piGpuIds[0]);
	//-----------------------------------
	float fWaitTime = 0.0f;
	while(true)
	{	int iSize = m_aSaveQueue.size();
		if(iSize == 0)
		{	if(m_bStop) break;
			mWait(0.5f);
			fWaitTime += 0.5f;
			if(fWaitTime > 120.0f) break;
			else continue;
		}
		fWaitTime = 0.0f;
		//----------------	
		pthread_mutex_lock(&m_aMutex);
		m_pPackage = m_aSaveQueue.front();
		m_aSaveQueue.pop();
		pthread_mutex_unlock(&m_aMutex);
		//------------------------------
		if(m_gfImg == 0L) mInit(); 
		mSaveSums();
		mSaveStack();
		mSaveCtfStack();
		mSaveCtfFit();
		delete m_pPackage;
		m_pPackage = 0L;
	}
	mClean();
}

void CSaveSerialCryoEM::mInit(void)
{
	cudaMalloc(&m_gfImg, m_pPackage->m_pAlnSums->m_tFmBytes);
	Util::GFindMinMax2D* pGFindMinMax = new Util::GFindMinMax2D;
        Util::GCalcMoment2D* pGCalcMoment = new Util::GCalcMoment2D;
	pGFindMinMax->SetSize(m_pPackage->m_pAlnSums->m_aiStkSize, false);
	pGCalcMoment->SetSize(m_pPackage->m_pAlnSums->m_aiStkSize, false);
        m_pvGFindMinMax2D = pGFindMinMax;
        m_pvGCalcMoment2D = pGCalcMoment;
}

void CSaveSerialCryoEM::mClean(void)
{
	if(m_gfImg == 0L) return;
	cudaFree(m_gfImg); m_gfImg = 0L;
        delete (Util::GFindMinMax2D*)m_pvGFindMinMax2D;
	m_pvGFindMinMax2D = 0L;
        delete (Util::GCalcMoment2D*)m_pvGCalcMoment2D;
	m_pvGCalcMoment2D = 0L;
}

void CSaveSerialCryoEM::mSaveSums(void)
{
	printf("Saving motion corrected images ......\n");	
	char acOutFile[512] = {'\0'}, acSuffix[32] = {'\0'};
	DU::CAlnSums* pAlnSums = m_pPackage->m_pAlnSums;
	int* piStkSize = pAlnSums->m_aiStkSize;
	float fPixSize = pAlnSums->m_fPixSize;
	//-----------------
	for(int i=0; i<pAlnSums->m_aiStkSize[2]; i++)
	{	float* pfAlnSum = (float*)pAlnSums->GetFrame(i);
		if(pfAlnSum == 0L) continue;
		//----------------
		pAlnSums->GetFileExt(i, acSuffix);
		sGenOutputName(m_pPackage->m_pcSerial, acSuffix, 
		   ".mrc", acOutFile);
		mSaveImage(acOutFile, pfAlnSum, piStkSize, fPixSize);
	}
	printf("Motion corrected images saved.\n\n");
}

void CSaveSerialCryoEM::mSaveStack(void)
{
	DU::CMrcStack* pAlnStack = m_pPackage->m_pAlnStack;
	if(pAlnStack == 0L) return;
	int iNumFrames = pAlnStack->m_aiStkSize[2];
	if(iNumFrames <= 0) return;
	//-------------------------
	char acMrcFile[256] = {'\0'}, acSuffix[16] = {'\0'};
	strcpy(acSuffix, "_Stk");
	sGenOutputName(m_pPackage->m_pcSerial, "_Stk", ".mrc", acMrcFile);
	//-----------------
	Mrc::CSaveMrc aSaveMrc;
	if(!aSaveMrc.OpenFile(acMrcFile)) return;
	//---------------------------------------
	printf("Saving motion corrected stack ......\n");
	aSaveMrc.SetMode(Mrc::eMrcFloat);
	aSaveMrc.SetImgSize(pAlnStack->m_aiStkSize, iNumFrames, 
	   1, pAlnStack->m_fPixSize);
	aSaveMrc.SetExtHeader(0, 32, 0);
	//------------------------------
	cudaStream_t stream;
	cudaStreamCreate(&stream);
	//------------------------
	Util::GFindMinMax2D *pGFindMin = 0L, *pGFindMax = 0L;
	pGFindMin = (Util::GFindMinMax2D*)m_pvGFindMinMax2D;
	pGFindMax = new Util::GFindMinMax2D;
	pGFindMax->SetSize(pAlnStack->m_aiStkSize, false);
	//------------------------------------------------
	Util::GCalcMoment2D* pGCalcMoment = (Util::GCalcMoment2D*)
	   m_pvGCalcMoment2D;
	//-------------------
	float fMin = (float)1e20, fMax = (float)-1e20;
	double dMean = 0.0;
	//-----------------
	int iLast = iNumFrames - 1;
	for(int i=0; i<iNumFrames; i++)
	{	void* pvFrame = pAlnStack->GetFrame(i);
		cudaMemcpy(m_gfImg, pvFrame, pAlnStack->m_tFmBytes,
                   cudaMemcpyDefault);
                pGFindMin->DoMin(m_gfImg, false, stream);
                pGFindMax->DoMax(m_gfImg, false, stream);
                pGCalcMoment->DoIt(m_gfImg, 1, false);
                //------------------------------------
                aSaveMrc.m_pSaveExt->SetPixelSize(i, pAlnStack->m_fPixSize);
		aSaveMrc.DoIt(i, pvFrame);
		cudaStreamSynchronize(stream);
		//----------------------------
		float ffMin = pGFindMin->GetResult();
		float ffMax = pGFindMax->GetResult();
		if(fMin > ffMin) fMin = ffMin;
		if(fMax < ffMax) fMax = ffMax;
		dMean += pGCalcMoment->GetResult();
		if(i % 10 != 0 && i != iLast) continue;
		printf("...... Frame %4d save, %4d frames left.\n",
		   i + 1, iLast - i);
	}
	dMean = dMean / pAlnStack->m_aiStkSize[2];
	//----------------------------------------
	aSaveMrc.SaveMinMaxMean(fMin, fMax, (float)dMean);
        aSaveMrc.CloseFile();
	printf("Motion corrected stack done.\n\n");
	//-----------------------------------------
	delete pGFindMax;
	cudaFree(stream);
}

void CSaveSerialCryoEM::mSaveImage
(	char* pcMrcFile,
	float* pfImg,
	int* piImgSize,
	float fPixSize
)
{	if(pfImg == 0L) return;
	//---------------------
	Mrc::CSaveMrc aSaveMrc;
	if(!aSaveMrc.OpenFile(pcMrcFile)) return;
	//---------------------------------------	
	aSaveMrc.SetMode(Mrc::eMrcFloat);
	aSaveMrc.SetImgSize(piImgSize, 1, 1, fPixSize);
	aSaveMrc.SetExtHeader(0, 0, 0);
	//-----------------------------
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
	aSaveMrc.CloseFile();
	//-------------------
	printf("Corrected sum has been saved in:\n");
	printf("   %s\n\n", pcMrcFile);
}

void CSaveSerialCryoEM::mSaveCtfStack(void)
{
	if(m_pPackage->m_pCtfStack == 0L) return;
	//-----------------
	char acMrcFile[256] = {'\0'};
        sGenOutputName(m_pPackage->m_pcSerial, "_Ctf", ".mrc", acMrcFile);
	//-----------------
	Mrc::CSaveMrc aSaveMrc;
        if(!aSaveMrc.OpenFile(acMrcFile)) return;
	//-----------------
	DU::CMrcStack* pCtfStack = m_pPackage->m_pCtfStack;
	aSaveMrc.SetMode(Mrc::eMrcFloat);
        aSaveMrc.SetImgSize(pCtfStack->m_aiStkSize, 
	   pCtfStack->m_aiStkSize[2], 1, 1.0f);
	//-----------------
	for(int i=0; i<pCtfStack->m_aiStkSize[2]; i++)
	{	void* pvImg = pCtfStack->GetFrame(i);
		aSaveMrc.DoIt(i, pvImg);
	}
}

void CSaveSerialCryoEM::mSaveCtfFit(void)
{
	if(m_pPackage->m_pvCtfParam == 0L) return;
	//-----------------
	char acTxtFile[256] = {'\0'};
	sGenOutputName(m_pPackage->m_pcSerial, "_Ctf", ".txt", acTxtFile);
	//-----------------
	FILE* pFile = fopen(acTxtFile, "w");
	if(pFile == 0L) return;
	//---------------------
	FindCtf::CCtfParam* pCtfParam = 
	   (FindCtf::CCtfParam*)m_pPackage->m_pvCtfParam;
	bool bAngstrom = true, bDegree = true;
	//-----------------
	fprintf(pFile, "# Columns: #1 micrograph number; "
	   "#2 - defocus 1 [A]; #3 - defocus 2; "
	   "#4 - azimuth of astigmatism; "
	   "#5 - additional phase shift [radian]; "
	   "#6 - cross correlation; "
	   "#7 - spacing (in Angstroms) up to which CTF rings were "
	   "fit successfully\n");
	fprintf(pFile, " %9.2f  %9.2f  %7.2f  %7.2f  "
	   "%9.5f  %8.4f\n", 
	   pCtfParam->GetDfMax(bAngstrom),
	   pCtfParam->GetDfMin(bAngstrom),
	   pCtfParam->GetAstAng(bDegree),
	   pCtfParam->GetExtPhase(bDegree),
	   pCtfParam->m_fScore,
	   10.0f); // 10.0f temporary
	fclose(pFile);
}	
