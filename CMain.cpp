#include "CMainInc.h"
#include "Align/CAlignInc.h"
#include "Util/CUtilInc.h"
#include "DataUtil/CDataUtilInc.h"
#include "TiffUtil/CTiffFileInc.h"
#include "EerUtil/CEerUtilInc.h"
#include "FindCtf/CFindCtfInc.h"
#include <Util/Util_Time.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

using namespace MotionCor2;
namespace DU = MotionCor2::DataUtil;
namespace TU = MotionCor2::TiffUtil;
namespace EU = MotionCor2::EerUtil;
namespace MU = MotionCor2::MrcUtil;

static DU::CStackFolder* s_pStackFolder = 0L;
static DU::CDataPackage* s_pLoadedPackage = 0L;

static void sDeletePackage(void)
{	if(s_pLoadedPackage == 0L) return;
	delete s_pLoadedPackage;
	s_pLoadedPackage = 0L;
}

CMain::CMain(void)
{
}

CMain::~CMain(void)
{
}

bool CMain::DoIt(void)
{
	//------------------------------------------------------------
	// read a specified folder to determine if there are any
	// movies to be processed.
	//------------------------------------------------------------
	s_pStackFolder = DU::CStackFolder::GetInstance();
	bool bSuccess = s_pStackFolder->ReadFiles();
	//-----------------
	if(!bSuccess)
	{	fprintf(stderr, "Error: no input image files are found.\n\n");
		return false;
	}
	//-----------------
	DU::CReadFmIntFile *pReadFmIntFile = 
	   DU::CReadFmIntFile::GetInstance();
	pReadFmIntFile->DoIt();
	//------------------------------------------------------------
	// load gain and/or dark references.
	//------------------------------------------------------------
	m_bHasGainFromFile = mLoadRefs();
	//------------------------------------------------------------
	// determine if CTF estimation will be done.
	//------------------------------------------------------------
	FindCtf::CFindCtfMain::bEstimate();
	//------------------------------------------------------------
	// configure buffer pool but delay GPU/CPU mem allocation
	// until movie files are being loaded.
	//------------------------------------------------------------
	CInput* pInput = CInput::GetInstance();
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetGpus(pInput->m_piGpuIds, pInput->m_iNumGpus);
	//------------------------------------------------------------
	// start single or sequential processing depending on 
	// number of movies in the specified folder.
	//------------------------------------------------------------
	if(pInput->m_iSerial >= 1) bSuccess = mDoSerialCryoEM();
	else bSuccess = mDoSingleCryoEM();
	//------------------------------------------------------------
	// wait a certain amount of time for CStackFolder thread to
	// exit, which means that no new movies have been saved, an
	// indication either all movies have been processed or no
	// new movies have been saved.
	//------------------------------------------------------------
	s_pStackFolder->WaitForExit(120.0f);
	m_aProcessThread.WaitForExit(120.0f);
	CSaveSerialCryoEM* pSaveSerial = CSaveSerialCryoEM::GetInstance();
	pSaveSerial->WaitForExit(120.0f);
	//---------------------------------
	s_pStackFolder = 0L;
	if(pInput->m_iSerial != 0) CBufferPool::DeleteInstance();
	DU::CStackFolder::DeleteInstance();
	DU::CReadFmIntFile::DeleteInstance();;
	return bSuccess;
}

bool CMain::mDoSingleCryoEM(void)
{
	CInput* pInput = CInput::GetInstance();
	char* pcInTiff = pInput->m_acInTifFile;
	int aiStkSize[3] = {0};
	//----------------------------------------------------------
	// Asynchronous load of movie stack. As soon as we know the
	// the movie size, we can start to allocate buffers.
	//----------------------------------------------------------
	if(pInput->IsInTiff())
	{	TU::CLoadTiffStack::OpenFile(aiStkSize);
		TU::CLoadTiffStack::AsyncLoad();
	}
	else if(pInput->IsInEer())
	{	EU::CLoadEerMain::OpenFile(aiStkSize);
		EU::CLoadEerMain::AsyncLoad();
	}
	else if(pInput->IsInMrc())
	{	MU::CLoadCryoEMStack::OpenFile(aiStkSize);
		MU::CLoadCryoEMStack::AsyncLoad();
	}
	//-------------------------------------------------------
	// Create CPU/GPU buffer while the movie is being loaded.
	//-------------------------------------------------------	
	mCreateBufferPool(aiStkSize, 0);
	//-----------------
	CLoadRefs* pLoadRefs = CLoadRefs::GetInstance();
        pLoadRefs->AugmentRefs(aiStkSize);
	//-----------------
	CGenStarFile* pGenStarFile = CGenStarFile::GetInstance();
	pGenStarFile->OpenFile(pcInTiff);
        pGenStarFile->SetStackSize(aiStkSize);
	//-----------------
	bool bLoaded = true;
	if(pInput->IsInTiff())
	{	s_pLoadedPackage = (DU::CDataPackage*)
		   TU::CLoadTiffStack::GetPackage();
	}
	else if(pInput->IsInEer())
	{	s_pLoadedPackage = (DU::CDataPackage*)
		   EU::CLoadEerMain::GetPackage();
	}
	else if(pInput->IsInMrc())
	{	s_pLoadedPackage = (DU::CDataPackage*)
		   MU::CLoadCryoEMStack::GetPackage();	
	}
	if(s_pLoadedPackage == 0L) return false;	
	//-----------------
	mOpenInAlnFile();
	mOpenOutAlnFile();
	//-----------------
	m_aProcessThread.DoIt(s_pLoadedPackage);
	s_pLoadedPackage = 0L;
	mWaitSaveThread();
	//-----------------
	pGenStarFile->CloseFile();
	return true;
}

bool CMain::mDoSerialCryoEM(void)
{
	CInput* pInput = CInput::GetInstance();
	char* pcPrefix = 0L;
        if(pInput->IsInMrc()) pcPrefix = pInput->m_acInMrcFile;
	else if(pInput->IsInTiff()) pcPrefix = pInput->m_acInTifFile;
	else pcPrefix = pInput->m_acInEerFile;
	//-----------------
	if(pInput->IsInTiff()) mDoSerialCryoEMTiff();
	else if(pInput->IsInEer()) mDoSerialCryoEMEer();
	else mDoSerialCryoEMMrc();	
	//-----------------
	mWaitProcessThread();
	CSaveSerialCryoEM* pSaveSerialCryoEM = 
	   CSaveSerialCryoEM::GetInstance();
	pSaveSerialCryoEM->WaitForExit(100000.0f);
	//-----------------
	return true;
}

void CMain::mDoSerialCryoEMMrc(void)
{
	int iCount = 0;
	//---------------------------------------------------------------
	// If the file queue is empty, check if the CStackFolder thread
	// is still alive. If so, it is still monitoring the folder for
	// new movies. Then we need to continue waiting.
	//---------------------------------------------------------------
	while(true)
	{	int iSize = s_pStackFolder->GetQueueSize();
		//--------------------------------------------------------
		// Wait 2 seconds to see if the folder monitoring thread
		// exits. If so, it means there is no more movie files.
		//-------------------------------------------------------
		if(iSize == 0)
		{	if(s_pStackFolder->WaitForExit(2.0f)) break;
			else continue;
		} 
		//---------------------------------------------------
		// We start loading when the previous saving is done
		//---------------------------------------------------
		mWaitSaveThread();
		int iErr = mAsyncLoad(iCount);
		if(iErr == 1) continue;
		//-------------------------------------------------------
		// Here is the overlapping of loading with processing
		//-------------------------------------------------------
		s_pLoadedPackage = (DU::CDataPackage*)
		   MU::CLoadCryoEMStack::GetPackage();
		//----------------
		if(!iErr == 2) sDeletePackage();
		if(s_pLoadedPackage == 0L) continue;
		//----------------
		mOpenInAlnFile();
		mOpenOutAlnFile();
		//----------------------------------------------
		// push the new package to the processing queue.
		//----------------------------------------------
		m_aProcessThread.AsyncDoIt(s_pLoadedPackage);
		s_pLoadedPackage = 0L;
		iCount += 1;
	}
	m_aProcessThread.Stop();
}

void CMain::mDoSerialCryoEMTiff(void)
{
	int iCount = 0;
	CGenStarFile* pGenStarFile = CGenStarFile::GetInstance();
	//---------------------------------------------------------------
	// If the file queue is empty, check if the CStackFolder thread
	// is still alive. If so, it is still monitoring the folder for
	// new movies. Then we need to continue waiting.
	//---------------------------------------------------------------
	while(true)	
	{	int iSize = s_pStackFolder->GetQueueSize();
		if(iSize == 0)
		{	if(s_pStackFolder->WaitForExit(2.0f)) break;
			else continue;
		}
		//---------------------------------------------------
		// We start loading when the previous saving is done.
		//---------------------------------------------------
		mWaitSaveThread();
		int iErr = mAsyncLoad(iCount);
		if(iErr == 1) continue;
		//-------------------------------------------------------
		// Here is the overlapping of loading with processing.
		//-------------------------------------------------------
		s_pLoadedPackage = (DU::CDataPackage*)
		   TU::CLoadTiffStack::GetPackage();
		if(iErr == 2) sDeletePackage();
		if(s_pLoadedPackage == 0L) continue;
		//----------------------------------
		pGenStarFile->OpenFile(s_pLoadedPackage->m_pcInFileName);
		pGenStarFile->SetStackSize(
		   s_pLoadedPackage->m_pRawStack->m_aiStkSize);
		//----------------
		mOpenInAlnFile();
		mOpenOutAlnFile();
		//----------------
		m_aProcessThread.AsyncDoIt(s_pLoadedPackage);
		s_pLoadedPackage = 0L;
		iCount += 1;
	}
	m_aProcessThread.Stop();
}

void CMain::mDoSerialCryoEMEer(void)
{
	int iCount = 0;
	CLoadRefs* pLoadRefs = CLoadRefs::GetInstance();
	//-----------------
	while(true)
	{	int iSize = s_pStackFolder->GetQueueSize();
		if(iSize == 0)
		{	if(s_pStackFolder->WaitForExit(2.0f)) break;
			else continue;
		}
		//---------------------------------------------------
		// Start loading when the previous saving is done.
		//---------------------------------------------------
		mWaitSaveThread();
		int iErr = mAsyncLoad(iCount);
		if(iErr == 1) continue;
		//----------------------------------------------------
		// Here is the overlapping of loading with processing
		//----------------------------------------------------
                s_pLoadedPackage = (DU::CDataPackage*)EU::
		   CLoadEerMain::GetPackage();
		if(iErr == 2) sDeletePackage();
		if(s_pLoadedPackage == 0L) continue;	
		//----------------
		mOpenInAlnFile();
		mOpenOutAlnFile();
                //----------------
		m_aProcessThread.AsyncDoIt(s_pLoadedPackage);
		s_pLoadedPackage = 0L;
		iCount += 1;
	}
	m_aProcessThread.Stop();
}

int CMain::mAsyncLoad(int iCount)
{
	CInput* pInput = CInput::GetInstance();
	int aiStkSize[3] = {0};
	bool bSuccess = true;
	//-----------------
	if(pInput->IsInTiff())
	{	bSuccess = TiffUtil::CLoadTiffStack::OpenFile(aiStkSize);
		if(bSuccess) TU::CLoadTiffStack::AsyncLoad();
	}
	else if(pInput->IsInEer())
	{	bSuccess = EU::CLoadEerMain::OpenFile(aiStkSize);
		if(bSuccess) EU::CLoadEerMain::AsyncLoad();
	}
	else
	{	bSuccess = MU::CLoadCryoEMStack::OpenFile(aiStkSize);
		if(bSuccess) MU::CLoadCryoEMStack::OpenFile(aiStkSize);
	}
	if(!bSuccess) return 1;  // open-file error
	//-----------------
	bSuccess = mCreateBufferPool(aiStkSize, iCount);
	if(!bSuccess) return 2;  // buffer err
	//-----------------
	if(pInput->IsInEer())
	{	CLoadRefs* pLoadRefs = CLoadRefs::GetInstance();
		pLoadRefs->AugmentRefs(aiStkSize);	
	}
	return 0;           // no error;
}

bool CMain::mLoadRefs(void)
{
	CInput* pInput = CInput::GetInstance();
	CLoadRefs* pLoadRefs = CLoadRefs::GetInstance();
	//-----------------
	bool bLoadGain = pLoadRefs->LoadGain(pInput->m_acGainMrc);
	bool bLoadDark = pLoadRefs->LoadDark(pInput->m_acDarkMrc);
	pLoadRefs->PostProcess(pInput->m_iRotGain, 
	   pInput->m_iFlipGain, pInput->m_iInvGain);
	//-----------------
	if(bLoadGain) printf("Gain reference has been loaded.\n");
	else printf("Gain reference not found.\n");
	//-----------------
	if(bLoadDark) printf("Dark reference has been loaded.\n\n");
	else printf("DarkReference not found.\n\n");
	//-----------------
	return bLoadGain;
}

bool CMain::mWaitProcessThread(void)
{
	int iNumTimes = 10000;
	for(int i=0; i<iNumTimes; i++)
	{	bool bExit = m_aProcessThread.WaitForExit(1.0f);
		if(!bExit) continue;
		return true;
	}
	return false;
}

bool CMain::mWaitSaveThread(void)
{
	int iNumTimes = 10000;
	CSaveSerialCryoEM* pSaveSerial = CSaveSerialCryoEM::GetInstance();
	for(int i=0; i<iNumTimes; i++)
	{	bool bExit = pSaveSerial->WaitForExit(1.0f);
		if(!bExit) continue;
		else return true;
	}
	return false;
}

void CMain::mOpenOutAlnFile(void)
{
	CInput* pInput = CInput::GetInstance();
	if(strlen(pInput->m_acOutAlnFolder) == 0) return;
	//-----------------------------------------------
	char* pcFileName = strrchr(s_pLoadedPackage->m_pcInFileName, '/');
	if(pcFileName != 0L) pcFileName += 1;
	else pcFileName = s_pLoadedPackage->m_pcInFileName;
	//-------------------------------------------------
	Align::CSaveAlign* pSaveAlign = Align::CSaveAlign::GetInstance();
	pSaveAlign->Open(pInput->m_acOutAlnFolder, pcFileName);
	pSaveAlign->SaveSetting(s_pLoadedPackage->m_pRawStack->m_aiStkSize, 
	   pInput->m_aiNumPatches, pInput->m_aiThrow);
}

void CMain::mOpenInAlnFile(void)
{
	CInput* pInput = CInput::GetInstance();
	if(strlen(pInput->m_acInAlnFolder) == 0) return;
	//----------------------------------------------
	char* pcFileName = strrchr(s_pLoadedPackage->m_pcInFileName, '/');
	if(pcFileName != 0L) pcFileName += 1;
	else pcFileName = s_pLoadedPackage->m_pcInFileName;
	//-------------------------------------------------
	Align::CLoadAlign* pLoadAlign = Align::CLoadAlign::GetInstance();
	pLoadAlign->Open(pInput->m_acInAlnFolder, pcFileName);
	if(!pLoadAlign->IsLoaded()) return;
	//---------------------------------
	memcpy(pInput->m_aiThrow, pLoadAlign->m_aiThrow, sizeof(int) * 2);
	memcpy(pInput->m_aiNumPatches, pLoadAlign->m_aiPatches,
	   sizeof(int) * 3);
}

bool CMain::mCreateBufferPool(int* piMrcSize, int iCount)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	if(iCount == 0)
	{	pBufferPool->CreateBuffers(piMrcSize);
		return true;
	}
	//-----------------
	if(piMrcSize[0] != pBufferPool->m_aiStkSize[0]) return false;
	if(piMrcSize[1] != pBufferPool->m_aiStkSize[1]) return false;
	if(piMrcSize[2] == pBufferPool->m_aiStkSize[2]) return true;
	//------------------------------------------------------
	// Wait for the processing thread to complete since the
	// buffer pool is shared object.
	//------------------------------------------------------
	m_aProcessThread.Stop();
	m_aProcessThread.WaitForExit(-1.0f);
	//--------------------------
	// Now it is safe to adjust
	//--------------------------
	pBufferPool->CreateBuffers(piMrcSize);
	pBufferPool->AdjustBuffer(piMrcSize[2]);

	printf("******** Num Frames: %d\n", piMrcSize[2]);

	return true;
}

