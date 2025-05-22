#include "CMainInc.h"
#include "Util/CUtilInc.h"
#include "DataUtil/CDataUtilInc.h"
#include "BadPixel/CDetectMain.h"
#include "BadPixel/CCorrectMain.h"
#include "Align/CAlignInc.h"
#include "Correct/CCorrectInc.h"
#include <Util/Util_Time.h>
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
namespace DU = MotionCor2::DataUtil;
namespace MU = MotionCor2::MrcUtil;


CProcessThread::CProcessThread(void)
{
	m_pvPackage = 0L;
}

CProcessThread::~CProcessThread(void)
{
}

void CProcessThread::DoIt(void* pvNewPackage)
{
	CInput* pInput = CInput::GetInstance();
	cudaSetDevice(pInput->m_piGpuIds[0]);
	m_pvPackage = pvNewPackage;
	mProcess();
	m_pvPackage = 0L;
}

void CProcessThread::AsyncDoIt(void* pvNewPackage)
{	
	pthread_mutex_lock(&m_aMutex);
	m_aProcQueue.push(pvNewPackage);	
	pthread_mutex_unlock(&m_aMutex);
	//------------------------------
	if(!IsAlive()) this->Start();
}

void CProcessThread::ThreadMain(void)
{
	float fWaitTime = 0.0f;
	CInput* pInput = CInput::GetInstance();
	cudaSetDevice(pInput->m_piGpuIds[0]);
	//-----------------------------------
	while(true)
	{	int iSize = m_aProcQueue.size();
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
		m_pvPackage = m_aProcQueue.front();
		m_aProcQueue.pop();
		pthread_mutex_unlock(&m_aMutex);
		//-----------------------------------------------
		// After processing, m_pvPackage has been pushed
		// into the saving queue in CSaveSerialCryoEM. 
		// So we set m_pvPackage to 0L.
		//-----------------------------------------------
		mProcess();
		m_pvPackage = 0L;
	}
	//-----------------
	CSaveSerialCryoEM* pSaveSerial = CSaveSerialCryoEM::GetInstance();
	pSaveSerial->Stop();
}

void CProcessThread::mProcess(void)
{
	Util_Time aTimer;
	aTimer.Measure();
	//---------------------------
	DU::CDataPackage* pPackage = (DU::CDataPackage*)m_pvPackage;
	DU::CSaveMovieDone* pSaveMovieDone = 
	   DU::CSaveMovieDone::GetInstance();
	pSaveMovieDone->DoStart(pPackage->m_pcInFileName);
	//---------------------------
	bool bSuccess = mCheckGain();
	if(!bSuccess) return;
	//---------------------------
	nvtxRangePushA("mApplyRefs");
	mApplyRefs();
	nvtxRangePop();
	//---------------------------
	nvtxRangePushA("mDetectBadPixels");
	mDetectBadPixels();
	nvtxRangePop();
	nvtxRangePushA("mCorrectBadPixels");
	mCorrectBadPixels();
	nvtxRangePop();
	nvtxRangePushA("mAlignStack");
	mAlignStack();
	nvtxRangePop();
	float fSecs = aTimer.GetElapsedSeconds();
	printf("Computation time: %f sec\n\n", fSecs);
	//---------------------------
	mSaveAlnSums();
	pSaveMovieDone->DoEnd(pPackage->m_pcInFileName);
}

bool CProcessThread::mCheckGain(void)
{
	CLoadRefs* pLoadRefs = CLoadRefs::GetInstance();
	if(pLoadRefs->m_pfGain == 0L)
	{	printf("Warning: Gain ref not found.\n"
		   "......   Gain correction will be skipped.\n\n");
		return true;
	}
	//-----------------
	DU::CDataPackage* pPackage = (DU::CDataPackage*)m_pvPackage;
	if(pLoadRefs->m_aiRefSize[0] == pPackage->m_pRawStack->m_aiStkSize[0]
        && pLoadRefs->m_aiRefSize[1] == pPackage->m_pRawStack->m_aiStkSize[1])
	{	return true;
	}
	//-----------------
	fprintf(stderr,
	   "Error: Gain and frame dimensions mismatch!\n"
	   "...... Gain size:  %d  %d\n"
	   "...... Frame size: %d  %d\n"
	   "Motion correction skipped.\n\n",
	   pLoadRefs->m_aiRefSize[0], pLoadRefs->m_aiRefSize[1],
	   pPackage->m_pRawStack->m_aiStkSize[0], 
	   pPackage->m_pRawStack->m_aiStkSize[1]);
	//-----------------
	return false;
}

void CProcessThread::mApplyRefs(void)
{
	Util_Time aTimer; aTimer.Measure();
	//---------------------------------
	CLoadRefs* pLoadRefs = CLoadRefs::GetInstance();
	DU::CDataPackage* pPackage = (DU::CDataPackage*)m_pvPackage;
	//----------------------------------------------------------
	MU::CApplyRefs applyRefs;
	applyRefs.DoIt(pPackage->m_pRawStack, pLoadRefs->m_pfGain, 
	   pLoadRefs->m_pfDark);
	//----------------------
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iSerial == 0) pLoadRefs->CleanRefs();
	//------------------------------------------------
	float fTime = aTimer.GetElapsedSeconds();
	printf("Apply gain: %.2f s\n\n", fTime);
}

bool CProcessThread::mDetectBadPixels(void)
{
	CInput* pInput = CInput::GetInstance();
	BadPixel::CDetectMain* pDetectMain = 0L;
	pDetectMain = BadPixel::CDetectMain::GetInstance();
	pDetectMain->DoIt();
	return true;
}

bool CProcessThread::mCorrectBadPixels(void)
{
	BadPixel::CDetectMain* pDetectMain = 0L;
        pDetectMain = BadPixel::CDetectMain::GetInstance();
	bool bClean = true;
	unsigned char* gucBadMap = pDetectMain->GetDefectMap(bClean);
	BadPixel::CDetectMain::DeleteInstance();
	//--------------------------------------
	int iDefectSize = 7;
	BadPixel::CCorrectMain aCorrectMain;
	aCorrectMain.DoIt(iDefectSize);
	//-----------------------------
	return true;
}

void CProcessThread::mAlignStack(void)
{
	Align::CAlignMain aAlignMain;
	aAlignMain.DoIt((DU::CDataPackage*)m_pvPackage);
}

void CProcessThread::mSaveAlnSums(void)
{
	CSaveSerialCryoEM* pSaveSerial = CSaveSerialCryoEM::GetInstance();
	pSaveSerial->AsyncSave((DU::CDataPackage*)m_pvPackage);
}
