#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <Util/Util_Time.h>
#include <memory.h>
#include <string.h>
#include <sys/sysinfo.h>
#include <string.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2;
using namespace MotionCor2::MrcUtil;
namespace DU = MotionCor2::DataUtil;

static int s_iMode = 0;
static DU::CStackFolder* s_pStackFolder = 0L;
static DU::CDataPackage* s_pPackage = 0L;
static DU::CFmIntParam* s_pFmIntParam = 0L;
static CLoadCryoEMStack* s_pLoadCryoEMStack = 0L;

static void sDeletePackage(void)
{	
	if(s_pPackage == 0) return;
	delete s_pPackage;
	s_pPackage = 0L;
}

CLoadCryoEMStack::CLoadCryoEMStack(void)
{
}

CLoadCryoEMStack::~CLoadCryoEMStack(void)
{
}

void CLoadCryoEMStack::Clean(void)
{
	if(s_pLoadCryoEMStack != 0L) delete s_pLoadCryoEMStack;
	s_pLoadCryoEMStack = 0L;
}

bool CLoadCryoEMStack::OpenFile(int aiStkSize[3])
{
	CInput* pInput = CInput::GetInstance();
	//----------------------------------------------------------
	// 1) Move the file name and serial from the file package to
	// the load package. Then eelete the file package.
	//----------------------------------------------------------
	bool bPop = true, bClean = true;
	s_pStackFolder = DU::CStackFolder::GetInstance();
	s_pPackage = s_pStackFolder->GetPackage(bPop);
	//--------------------------------------------
	Mrc::CLoadMrc loadMrc;
	if(!loadMrc.OpenFile(s_pPackage->m_pcInFileName))
	{	fprintf(stderr, "CLoadCryoEMStack::OpenFile: "
		   "cannot open stack file, skip,\n"
		   "   %s\n\n", s_pPackage->m_pcInFileName);
		sDeletePackage(); return false;
	}
	//------------------------------------------------
        s_iMode = loadMrc.m_pLoadMain->GetMode();
	loadMrc.m_pLoadMain->GetSize(aiStkSize, 3);
	printf("MRC file size mode: %d  %d  %d  %d\n", 
	   aiStkSize[0], aiStkSize[1], aiStkSize[2], s_iMode);
	//----------------------------------------------------------
	// If s_pPackage does not have CFmIntParam, this is a new
	// package and we create a new CFmIntParam object.
	//----------------------------------------------------------
	if(s_pPackage->m_pFmIntParam == 0L)
	{	s_pPackage->m_pFmIntParam = new DU::CFmIntParam;
	}
	s_pPackage->m_pFmIntParam->Setup(aiStkSize[2], s_iMode);
	aiStkSize[2] = s_pPackage->m_pFmIntParam->m_iNumIntFms;
	//-----------------------------------------------------------
	// If pPackage does not have CFmGrouParam, create one now.
	//-----------------------------------------------------------
	if(s_pPackage->m_pFmGroupParams == 0L)
	{	s_pPackage->m_pFmGroupParams = new DataUtil::CFmGroupParam[2];
	}
	s_pPackage->m_pFmGroupParams[0].Setup(pInput->m_aiGroup[0],
	   s_pPackage->m_pFmIntParam);
	s_pPackage->m_pFmGroupParams[1].Setup(pInput->m_aiGroup[1],
	   s_pPackage->m_pFmIntParam);
	//--------------------------------------------------------
	// Create the rendered stack if the data package 
	// does not have one in it.
	//----------------------------------------------
	if(s_pPackage->m_pRawStack == 0L)
	{	s_pPackage->m_pRawStack = new DataUtil::CMrcStack;
	}
	int iIntMode = s_iMode;
	if(s_pPackage->m_pFmIntParam->bIntegrate()) 
	{	iIntMode = Mrc::eMrcUChar;
	}
	s_pPackage->m_pRawStack->Create(iIntMode, aiStkSize);
	//---------------------------------------------------
	printf("Rendered size mode: %d  %d  %d  %d\n",
	   aiStkSize[0], aiStkSize[1], aiStkSize[2], iIntMode);	
	return true;
}

void CLoadCryoEMStack::AsyncLoad(void)
{
	if(s_pPackage == 0L) return;
	s_pFmIntParam = s_pPackage->m_pFmIntParam;
	//----------------------------------------
	CLoadCryoEMStack::Clean();
	s_pLoadCryoEMStack = new CLoadCryoEMStack;
	s_pLoadCryoEMStack->Start();
}

void* CLoadCryoEMStack::GetPackage(void)
{
	if(s_pPackage == 0L) return 0L;
	if(s_pLoadCryoEMStack == 0L) return 0L;
	//-------------------------------------
	s_pLoadCryoEMStack->WaitForExit(-1.0f);
	bool bLoaded = s_pLoadCryoEMStack->m_bLoaded;
	delete s_pLoadCryoEMStack;
	s_pLoadCryoEMStack = 0L;
	//----------------------
	if(!bLoaded) sDeletePackage();
	DU::CDataPackage* pRetPackage = s_pPackage;
	s_pPackage = 0L;
	return pRetPackage;
}

void CLoadCryoEMStack::ThreadMain(void)
{	
	Util_Time utilTime;
	utilTime.Measure();
	//-----------------
	m_bLoaded = false;
	m_pLoadMrc = new Mrc::CLoadMrc;
	m_pLoadMrc->OpenFile(s_pPackage->m_pcInFileName); 
	//-----------------
	if(s_pFmIntParam->bIntegrate()) mLoadInt(); 
	else mLoadSingle();
	//-----------------
	delete m_pLoadMrc;
	m_pLoadMrc = 0L;
	//-----------------
	char* pcFileName = strrchr(s_pPackage->m_pcInFileName, '/');
	if(pcFileName == 0L) pcFileName = s_pPackage->m_pcInFileName;
	else pcFileName = pcFileName + 1;
	//-----------------
	printf("*** %s: loaded in %.2f seconds.\n\n", pcFileName,
	   utilTime.GetElapsedSeconds());
	m_bLoaded = true;
}

void CLoadCryoEMStack::mLoadSingle(void)
{
	DataUtil::CMrcStack* pMrcStack = s_pPackage->m_pRawStack;
	for(int i=0; i<pMrcStack->m_aiStkSize[2]; i++)
	{	void* pvIntFm = pMrcStack->GetFrame(i);
		int iRawFm = s_pFmIntParam->GetIntFmStart(i);
		m_pLoadMrc->m_pLoadImg->DoIt(iRawFm, pvIntFm);
	}
}

void CLoadCryoEMStack::mLoadSingle4Bits(void)
{
	DataUtil::CMrcStack* pMrcStack = s_pPackage->m_pRawStack;
	int* piStkSize = pMrcStack->m_aiStkSize;
	//--------------------------------------
	int aiPkdSize[] = {(piStkSize[0] + 1) / 2, piStkSize[1]};
	int iPkdSize = aiPkdSize[0] * aiPkdSize[1];
	int iIntSize = pMrcStack->GetPixels();
	unsigned char* pucPkd = new unsigned char[iPkdSize];
	//--------------------------------------------------
	for(int i=0; i<pMrcStack->m_aiStkSize[2]; i++)
	{	int iPkdFm = s_pFmIntParam->GetIntFmStart(i);
		m_pLoadMrc->m_pLoadImg->DoIt(iPkdFm, pucPkd);
		void* pvIntFm = pMrcStack->GetFrame(i);
		Mrc::C4BitImage::Unpack(pucPkd, pvIntFm, piStkSize); 
	}
	delete[] pucPkd;
}

void CLoadCryoEMStack::mLoadInt(void)
{
	CInput* pInput = CInput::GetInstance();
	cudaSetDevice(pInput->m_piGpuIds[0]);
	//-----------------------------------
	if(s_iMode == Mrc::eMrc4Bits)
	{	mLoadInt4Bits();
		return;
	}
	//-------------
	DataUtil::CMrcStack* pMrcStack = s_pPackage->m_pRawStack;
	size_t tFmBytes = pMrcStack->m_tFmBytes;
	unsigned char *pucRaw = 0L, *gucSum = 0L;
	cudaMalloc(&gucSum, tFmBytes);
	//---------------------------------------------
	Util::GAddFrames aGAddFrames;
	for(int i=0; i<pMrcStack->m_aiStkSize[2]; i++)
	{	int iIntFmStart = s_pFmIntParam->GetIntFmStart(i);
		int iIntFmSize = s_pFmIntParam->GetIntFmSize(i);
		void* pvIntFm = pMrcStack->GetFrame(i);
		m_pLoadMrc->m_pLoadImg->DoIt(iIntFmStart, pvIntFm);
		if(iIntFmSize == 1) continue;
		//-------------------------------------------------
		pucRaw = (unsigned char*)pvIntFm;
		cudaMemcpy(gucSum, pucRaw, tFmBytes, cudaMemcpyDefault);
		for(int j=1; j<iIntFmSize; j++)
		{	m_pLoadMrc->m_pLoadImg->DoIt(iIntFmStart+j, pucRaw);
			aGAddFrames.DoIt(pucRaw, gucSum, gucSum,
			   pMrcStack->m_aiStkSize);
		}
		cudaMemcpy(pvIntFm, gucSum, tFmBytes, cudaMemcpyDefault);
	}
	cudaFree(gucSum);
}

void CLoadCryoEMStack::mLoadInt4Bits(void)
{
	DataUtil::CMrcStack* pMrcStack = s_pPackage->m_pRawStack;
        size_t tFmBytes = pMrcStack->m_tFmBytes;
	int* piRawSize = pMrcStack->m_aiStkSize;
	//--------------------------------------	
	int iPkdBytes = (piRawSize[0] + 1) / 2 * piRawSize[1] * sizeof(char);
	unsigned char *pucPkd = 0L, *gucPkd = 0L;
	cudaMallocHost(&pucPkd, iPkdBytes);
	cudaMalloc(&gucPkd, iPkdBytes);
	//----------------------------------------
	unsigned char *gucRaw = 0L, *gucSum = 0L;
	cudaMalloc(&gucRaw, tFmBytes);
	cudaMalloc(&gucSum, tFmBytes);
	//-------------------------------------------
	G4BitImage aG4BitImg;
	Util::GAddFrames aGAddFrms;
	//-----------------------------------------------------------------
	for(int i=0; i<pMrcStack->m_aiStkSize[2]; i++)
	{	int iIntFmStart = s_pFmIntParam->GetIntFmStart(i);
		int iIntFmSize = s_pFmIntParam->GetIntFmSize(i);
		//----------------------------------------------
		m_pLoadMrc->m_pLoadImg->DoIt(iIntFmStart, pucPkd);
		cudaMemcpy(gucPkd, pucPkd, iPkdBytes, cudaMemcpyDefault);
		aG4BitImg.Unpack(gucPkd, gucSum, piRawSize);
		//-------------------------------------------------------
		void* pvIntFm = pMrcStack->GetFrame(i);
		for(int j=1; j<iIntFmSize; j++)
		{	int k = iIntFmStart + j;
			m_pLoadMrc->m_pLoadImg->DoIt(k, pucPkd);
			cudaMemcpy(gucPkd, pucPkd, iPkdBytes,
			   cudaMemcpyDefault);
			aG4BitImg.Unpack(gucPkd, gucRaw, piRawSize);
			aGAddFrms.DoIt(gucRaw, gucSum, gucSum, piRawSize);
		}
		cudaMemcpy(pvIntFm, gucSum, tFmBytes, cudaMemcpyDefault);
	}
	cudaFree(gucRaw);
	cudaFree(gucPkd);
	cudaFree(gucSum);
	cudaFreeHost(pucPkd);
}

float CLoadCryoEMStack::mCalcMean(char* pcFrame)
{
	DataUtil::CMrcStack* pMrcStack = s_pPackage->m_pRawStack;
	int iMode = pMrcStack->m_iMode;
	int iPixels = pMrcStack->GetPixels(); 
	double dMean = 0.0;
	//-----------------
	if(iMode == Mrc::eMrc4Bits)
	{	unsigned char* pucBuf = new unsigned char[iPixels];
		Mrc::C4BitImage::Unpack(pcFrame, pucBuf,
			pMrcStack->m_aiStkSize);
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

