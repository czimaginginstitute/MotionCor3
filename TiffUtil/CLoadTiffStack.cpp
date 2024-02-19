#include "CTiffFileInc.h"
#include "../CMainInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <Util/Util_Time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/sysinfo.h>
#include <memory.h>
#include <string.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::TiffUtil;
namespace DU = MotionCor2::DataUtil;

static int s_iMode = 0;
static Util::CNextItem* s_pNextItem = 0L;
static int s_iNumThreads = 2;
static CLoadTiffStack* s_pLoadTiffStacks = 0L;
//--------------------------------------------
static DU::CStackFolder* s_pStackFolder = 0L;
static DU::CDataPackage* s_pPackage = 0L;
static DU::CFmIntParam* s_pFmIntParam = 0L;

static void sDeletePackage(void)
{
	if(s_pPackage == 0L) return;
	delete s_pPackage;
	s_pPackage = 0L;
}

CLoadTiffStack::CLoadTiffStack(void)
{
}

CLoadTiffStack::~CLoadTiffStack(void)
{
}

void CLoadTiffStack::Clean(void)
{
	if(s_pNextItem != 0L) delete s_pNextItem;
	if(s_pLoadTiffStacks != 0L) delete[] s_pLoadTiffStacks;
	s_pNextItem = 0L;
	s_pLoadTiffStacks = 0L;
}

bool CLoadTiffStack::OpenFile(int aiStkSize[3])
{
	CInput* pInput = CInput::GetInstance();
	//----------------------------------------------------------
	// Move the file name and serial from the file pacakge to
	// the load package. We are then done with the file package.
	//----------------------------------------------------------
	bool bPop = true, bClean = true;
	s_pStackFolder = DU::CStackFolder::GetInstance();
	s_pPackage = s_pStackFolder->GetPackage(bPop);
	//--------------------------------------------
	int iFile = open(s_pPackage->m_pcInFileName, O_RDONLY);
	if(iFile == -1)
	{	fprintf(stderr, "CLoadTiffStack: cannot open Tiff file, "
		   "skip,\n   %s\n\n", s_pPackage->m_pcInFileName);
		sDeletePackage(); 
		return false;
	}
	//---------------------------------------------------------
	CLoadTiffHeader aLoadHeader;
	aLoadHeader.DoIt(iFile);
	aLoadHeader.GetSize(aiStkSize, 3);
	s_iMode = aLoadHeader.GetMode();
	printf("TIFF file size mode: %d  %d  %d  %d\n",
	   aiStkSize[0], aiStkSize[1], aiStkSize[2], s_iMode);
	//----------------------------------------------------
	// If s_pPackage does not have m_pFmIntParam, this is
	// a new package amd we create an object
	//----------------------------------------------------
	if(s_pPackage->m_pFmIntParam == 0L)
	{	s_pPackage->m_pFmIntParam = new DataUtil::CFmIntParam;
	}
	s_pPackage->m_pFmIntParam->Setup(aiStkSize[2], s_iMode);
	aiStkSize[2] = s_pPackage->m_pFmIntParam->m_iNumIntFms;
	//-------------------------------------------------------------
	// If s_pPackage does not have m_pFmGroupParam, create one.
	//-------------------------------------------------------------
	if(s_pPackage->m_pFmGroupParams == 0L)
	{	s_pPackage->m_pFmGroupParams = new DataUtil::CFmGroupParam[2];
	}
	s_pPackage->m_pFmGroupParams[0].Setup(pInput->m_aiGroup[0],
	   s_pPackage->m_pFmIntParam);
	s_pPackage->m_pFmGroupParams[1].Setup(pInput->m_aiGroup[1],
	   s_pPackage->m_pFmIntParam);
	//-----------------------------------------------------------
	// Create the rendered stack if s_pPackage does not have one
	//-----------------------------------------------------------
	if(s_pPackage->m_pRawStack == 0L)
	{	s_pPackage->m_pRawStack = new DataUtil::CMrcStack;
	}
	int iIntMode = s_iMode;
	if(s_pPackage->m_pFmIntParam->bIntegrate()) 
	{	iIntMode = Mrc::eMrcUChar;
	}
	s_pPackage->m_pRawStack->Create(iIntMode, aiStkSize);
	//----------------------------------------------------
	printf("Rendered size mode: %d  %d  %d  %d\n\n", 
	  aiStkSize[0], aiStkSize[1], aiStkSize[2], s_iMode);
	close(iFile);
	return true;
}

void CLoadTiffStack::AsyncLoad(void)
{
	CLoadTiffStack::Clean();
	s_pFmIntParam = s_pPackage->m_pFmIntParam;
	s_pNextItem = new Util::CNextItem;
	s_pNextItem->Create(s_pPackage->m_pRawStack->m_aiStkSize[2]);
	s_pLoadTiffStacks = new CLoadTiffStack[s_iNumThreads];
	//----------------------------------------------------
	// start multiple threads to load the TIFF movie
	//----------------------------------------------------
	for(int i=0; i<s_iNumThreads; i++)
	{	int iFile = open(s_pPackage->m_pcInFileName, O_RDONLY);
		s_pLoadTiffStacks[i].Run(iFile);
	}
}

void* CLoadTiffStack::GetPackage(void)
{
	if(s_pLoadTiffStacks == 0L || s_pPackage == 0L) 
	{	CLoadTiffStack::Clean();
		return 0L;
	}
	//------------------------------
	float fLoadTime = 0.0f;
	bool bLoaded = true;
	for(int i=0; i<s_iNumThreads; i++)
	{	s_pLoadTiffStacks[i].WaitForExit(-1.0f);
		bLoaded = bLoaded && s_pLoadTiffStacks[i].m_bLoaded;
		//--------------------------------------------------
		if(s_pLoadTiffStacks[i].m_fLoadTime > fLoadTime)
		{	fLoadTime = s_pLoadTiffStacks[i].m_fLoadTime;
		}
	}
	CLoadTiffStack::Clean();
	printf("Load Tiff movie: %.2f seconds\n\n", fLoadTime);
	//----------------------------------------------------------
	// If loading fails, do not pop the package from load queue
	// since it is reused for the next load.
	//----------------------------------------------------------
	if(!bLoaded) sDeletePackage();
	DU::CDataPackage* pRetPackage = s_pPackage; s_pPackage = 0L;
	return pRetPackage;
}

void CLoadTiffStack::Run(int iFile)
{
	m_iFile = iFile;
	m_bLoaded = false;
	this->Start();
}

void CLoadTiffStack::ThreadMain(void)
{
	if(m_iFile == -1) return;
	//-----------------------
	Util_Time aTimer;
	aTimer.Measure();
	nvtxRangePushA("CLoadTiffStack");
	//-------------------------------
	m_pLoadTiffImage = new CLoadTiffImage;
	m_pLoadTiffImage->SetFile(m_iFile);
	//---------------------------------
	if(s_pPackage->m_pFmIntParam->bIntegrate()) 
	{	CInput* pInput = CInput::GetInstance();
		if(pInput->m_iSerial == 0) mLoadIntGpu();
		else mLoadIntCpu();
	}
	else mLoadSingle();
	//-----------------
	delete m_pLoadTiffImage;
	m_pLoadTiffImage = 0L;
	close(m_iFile);
	//-------------
	nvtxRangePop();
	m_fLoadTime = aTimer.GetElapsedSeconds();
}

void CLoadTiffStack::mLoadSingle(void)
{
	while(true)
	{	int iIntFm = s_pNextItem->GetNext();
		if(iIntFm < 0) break;
		//-------------------
		int iIntFmStart = s_pFmIntParam->GetIntFmStart(iIntFm);
		void* pvFrame = s_pPackage->m_pRawStack->GetFrame(iIntFm);
		m_bLoaded = m_pLoadTiffImage->DoIt(iIntFmStart, 
		   (char*)pvFrame);
		if(!m_bLoaded) break;
	}
}

void CLoadTiffStack::mLoadIntGpu(void)
{
	CInput* pInput = CInput::GetInstance();
	cudaSetDevice(pInput->m_piGpuIds[0]);
	//-----------------------------------
	DataUtil::CMrcStack* pRawStack = s_pPackage->m_pRawStack;
	unsigned char *gucRaw = 0L, *gucSum = 0L;
	size_t tFmBytes = pRawStack->m_tFmBytes;
	cudaMalloc(&gucSum, tFmBytes);
	cudaMalloc(&gucRaw, tFmBytes);
	//----------------------------
	while(true)
	{	int iIntFm = s_pNextItem->GetNext();
		if(iIntFm < 0) break;
		void* pvIntFm = pRawStack->GetFrame(iIntFm);
		//------------------------------------------
		int iIntFmStart = s_pFmIntParam->GetIntFmStart(iIntFm);
		int iIntFmSize = s_pFmIntParam->GetIntFmSize(iIntFm);
		m_bLoaded = m_pLoadTiffImage->DoIt(iIntFmStart, 
		   (char*)pvIntFm);
		//---------------------------------------------
		if(iIntFmSize == 1)
		{	if(!m_bLoaded) break;
			else continue;
		}
		//---------------------------
		cudaMemcpy(gucSum, pvIntFm, tFmBytes, cudaMemcpyDefault);
		Util::GAddFrames aGAddFrames;
		//---------------------------
		for(int i=1; i<iIntFmSize; i++)
		{	m_bLoaded = m_pLoadTiffImage->DoIt(
			   iIntFmStart+i, (char*)pvIntFm);
			if(!m_bLoaded) break;
			cudaMemcpy(gucRaw, pvIntFm, tFmBytes, 
			   cudaMemcpyDefault);
			aGAddFrames.DoIt(gucRaw, gucSum, gucSum,
			   pRawStack->m_aiStkSize);
		}
		if(!m_bLoaded) break;
		cudaMemcpy(pvIntFm, gucSum, tFmBytes, cudaMemcpyDefault);
	}
	//---------------------------------------------------------------
	cudaFree(gucRaw);
	cudaFree(gucSum);
}

void CLoadTiffStack::mLoadIntCpu(void)
{
	DataUtil::CMrcStack* pRawStack = s_pPackage->m_pRawStack;
	size_t tFmBytes = pRawStack->m_tFmBytes;
	int iPixels = pRawStack->GetPixels();
	unsigned char *pucInt = 0L, *pucRaw = 0L;
	pucRaw = new unsigned char[iPixels];
	while(true)
	{	int iIntFm = s_pNextItem->GetNext();
		if(iIntFm < 0) break;
		pucInt = (unsigned char*)pRawStack->GetFrame(iIntFm);
		//---------------------------------------------------
		int iIntFmStart = s_pFmIntParam->GetIntFmStart(iIntFm);
		int iIntFmSize = s_pFmIntParam->GetIntFmSize(iIntFm);
		m_bLoaded = m_pLoadTiffImage->DoIt(iIntFmStart, pucInt);
		//------------------------------------------------------
		if(iIntFmSize == 1)
		{	if(m_bLoaded) continue;
			else break;
		}
		//------------------------------
		for(int i=1; i<iIntFmSize; i++)
		{	int iRawFm = iIntFmStart + i;
			m_bLoaded = m_pLoadTiffImage->DoIt(iRawFm, pucRaw);
			if(!m_bLoaded) break;
			//-------------------
			for(int j=0; j<iPixels; j++)
			{	unsigned short usVal = pucRaw[j] +
				   (unsigned short)pucInt[j];
				pucInt[j] = (usVal < 255) ? usVal : 255;
			}
		}
		if(!m_bLoaded) break;
	}
	if(pucRaw != 0L) delete[] pucRaw;
}
