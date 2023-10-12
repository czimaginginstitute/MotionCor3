#include "CDetectMain.h"
#include "../CMainInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include "../TiffUtil/CTiffFileInc.h"
#include "../Util/CUtilInc.h"
#include <Util/Util_Time.h>
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cuda_runtime.h>

using namespace MotionCor2::BadPixel;
using namespace MotionCor2::Util;
using namespace MotionCor2::MrcUtil;

CDetectMain* CDetectMain::m_pInstance = 0L;

CDetectMain* CDetectMain::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CDetectMain;
	return m_pInstance;
}

void CDetectMain::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CDetectMain::CDetectMain(void)
{
	m_fThreshold = 6.0f;
	m_aiDefectSize[0] = 6;
	m_aiDefectSize[1] = 6;
}

CDetectMain::~CDetectMain(void)
{
}

//-------------------------------------------------------------------
// 1. pMrcStack must be gain applied before calling this
//    function.
// 2. We should have a main worker thread where gain is applied,
//    bad pixe is detected, corrected, and drift alignment and
//    correction are performed.
//-------------------------------------------------------------------
void CDetectMain::DoIt(void)
{
	printf("Detect bad and hot pixels.\n");
	//-------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	m_aiPadSize[0] = pSumBuffer->m_aiCmpSize[0] * 2;
	m_aiPadSize[1] = pSumBuffer->m_aiCmpSize[1];
	//------------------------------------------
	void* pvBuf = pBufferPool->GetPinnedBuf(0); // do not free
	memset(pvBuf, 0, sizeof(char) * m_aiPadSize[0] * m_aiPadSize[1]);
	m_pucBadMap = reinterpret_cast<unsigned char*>(pvBuf);
	//----------------------------------------------------
	MrcUtil::CSumFFTStack sumFFTStack;
	sumFFTStack.DoIt(EBuffer::frm, false);
	mDetectHot();
	//------------
	CGenStarFile* pGenStarFile = CGenStarFile::GetInstance();
	pGenStarFile->SetHotPixels(m_pucBadMap, m_aiPadSize, true);
	//---------------------------------------------------------
	mDetectPatch();
	cudaDeviceSynchronize();
	//----------------------
	mLoadDefectFile();
	mLoadDefectMap();
}

unsigned char* CDetectMain::GetDefectMap(bool bClean)
{
	unsigned char* pucBadMap = m_pucBadMap;
	if(bClean) m_pucBadMap = 0L;
	return pucBadMap;
}

void CDetectMain::mDetectPatch(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(0);
	//------------------------
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
	//-------------------------------------------------
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	cufftComplex* gCmpBuf1 = pTmpBuffer->GetFrame(0, 0);
	cufftComplex* gCmpBuf2 = pTmpBuffer->GetFrame(0, 1);
	//--------------------------------------------------
	float* gfPadSum = reinterpret_cast<float*>(gCmpSum);
	float* gfPadCC = reinterpret_cast<float*>(gCmpBuf1);
	float* gfPadBuf = reinterpret_cast<float*>(gCmpBuf2);
	//---------------------------------------------------
	GDetectPatch detectPatch;
	detectPatch.DoIt(gfPadSum, gfPadCC, gfPadBuf, m_pucBadMap,
	   m_aiPadSize, m_aiDefectSize, m_fThreshold);
}

void CDetectMain::mDetectHot(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
	//-------------------------------------------------
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	cufftComplex* gCmpBuf = pTmpBuffer->GetFrame(0, 0);
	float* gfPadSum = reinterpret_cast<float*>(gCmpSum);
	float* gfPadBuf = reinterpret_cast<float*>(gCmpBuf);
	//---------------------------------------------------
	GDetectHot aDetectHot;
        aDetectHot.DoIt(gfPadSum, gfPadBuf, m_aiPadSize, 
		m_fThreshold, m_pucBadMap);
}

void CDetectMain::mLabelDefects
(  	float* gfPadImg,
   	int* piDefects,
   	int iNumDefects
)
{	/*
	if(iNumDefects == 0) return;
	if(strlen(pInput->m_acTmpFile) == 0) return;
	//------------------------------------------
	GLabelDefect labelDefect;
	labelDefect.SetLabelSize(6);
	labelDefect.DoIt(gfImg, m_aiFrmSize, piDefects, iNumDefects);
	//-----------------------------------------------------------
	printf("Label defects in pre-correction image.\n");
	CInput* pInput = CInput::GetInstance();
	CSaveTempMrc aSTM;
	aSTM.SetFile(pInput->m_acTmpFile, "-Defect");
	aSTM.GDoIt(gfImg, 2, m_aiFrmSize);
	*/
}

void CDetectMain::mLoadDefectFile(void)
{
	CInput* pInput = CInput::GetInstance();
	if(strlen(pInput->m_acDefectFile) == 0) return;
	//---------------------------------------------
	FILE* pFile = fopen(pInput->m_acDefectFile, "rt");
	if(pFile == 0L) return;
	//---------------------
	printf("Load camera defects.\n");
	int iFrmSizeX = (m_aiPadSize[0] / 2 - 1) * 2;
	char acBuf[256] = {0};
	int aiEntry[4];
	//-------------
	while(!feof(pFile))
	{	fgets(acBuf, 256, pFile);
		int iItems = sscanf(acBuf, "%d  %d  %d  %d", aiEntry+0, 
			aiEntry+1, aiEntry+2, aiEntry+3);
		memset(acBuf, 0, sizeof(acBuf));
		if(iItems != 4) continue;
		//-----------------------
		printf("...... Defect: %6d  %6d  %6d  %6d\n",
			aiEntry[0], aiEntry[1], aiEntry[2], aiEntry[3]);
		//------------------------------------------------------
		if(aiEntry[0] < 0) aiEntry[0] = 0;
		if(aiEntry[1] < 0) aiEntry[1] = 0;
		int iEndX = aiEntry[0] + aiEntry[2];
		int iEndY = aiEntry[1] + aiEntry[3];
		if(iEndX > iFrmSizeX) iEndX = iFrmSizeX;
		if(iEndY > m_aiPadSize[1]) iEndY = m_aiPadSize[1];
		//------------------------------------------------
		for(int y=aiEntry[1]; y<iEndY; y++)
		{	int k = y * m_aiPadSize[0];
			for(int x=aiEntry[0]; x<iEndX; x++)
			{	m_pucBadMap[k+x] = 1;
			}
		}
	}
	fclose(pFile);
	printf("Load camera defects: done.\n\n");
}

void CDetectMain::mLoadDefectMap(void)
{
	CInput* pInput = CInput::GetInstance();
	if(strlen(pInput->m_acDefectMap) == 0) return;
	//--------------------------------------------
	bool bLoad = mLoadDefectMapMRC();
	if(bLoad) return;
	//---------------
	mLoadDefectMapTIFF();
}

bool CDetectMain::mLoadDefectMapMRC(void)
{
	CInput* pInput = CInput::GetInstance();
	Mrc::CLoadMrc aLoadMrc;
	if(!aLoadMrc.OpenFile(pInput->m_acDefectMap)) return false;
	//---------------------------------------------------------
	int iFrmSizeX = (m_aiPadSize[0] / 2 - 1) * 2;
	int iMode = aLoadMrc.m_pLoadMain->GetMode();
	if(iMode != Mrc::eMrcUChar && iMode != Mrc::eMrcUCharEM) return false;
	int iSizeX = aLoadMrc.m_pLoadMain->GetSizeX();
	int iSizeY = aLoadMrc.m_pLoadMain->GetSizeY();
	if(iSizeX != iFrmSizeX) return false;
	if(iSizeY != m_aiPadSize[1]) return false;
	//----------------------------------------
	int iPixels = iSizeX * iSizeY;
	unsigned char* pucMap = new unsigned char[iPixels];
	memset(pucMap, 0, iPixels * sizeof(char));
	aLoadMrc.m_pLoadImg->DoIt(0, pucMap);
	//-----------------------------------
	for(int y=0; y<m_aiPadSize[1]; y++)
	{	int iSrc = y * iSizeX;
		int iDst = y * m_aiPadSize[0];
		for(int x=0; x<iSizeX; x++)
		{	m_pucBadMap[iDst+x] = m_pucBadMap[iDst+x] 
				| (pucMap[iSrc+x] & 0x1);
		}
	}
	/*
	Util::CSaveTempMrc aSaveTmpMrc;
	aSaveTmpMrc.SetFile(pInput->m_acTmpFile, "-DefectMap.mrc");
	aSaveTmpMrc.DoIt(pucMap, 0, m_aiFrmSize);
	*/
	if(pucMap != 0L) delete[] pucMap;
	return true;
}

bool CDetectMain::mLoadDefectMapTIFF(void)
{
	CInput* pInput = CInput::GetInstance();
	int iFile = open(pInput->m_acDefectMap, O_RDONLY);
	if(iFile == -1) return false;
	//---------------------------
	TiffUtil::CLoadTiffImage aLoadTiff;
	if(!aLoadTiff.SetFile(iFile))
	{	close(iFile);
		return false;
	}
	//-------------------
	if(aLoadTiff.m_iMode != Mrc::eMrcUChar
		&& aLoadTiff.m_iMode != Mrc::eMrcUCharEM)
	{	close(iFile);
		return false;
	}
	//-------------------
	int iFrmSizeX = (m_aiPadSize[0] / 2 - 1) * 2;
	if(aLoadTiff.m_aiSize[0] != iFrmSizeX
		|| aLoadTiff.m_aiSize[1] != m_aiPadSize[1])
	{	close(iFile);
		return false;
	}
	//-------------------
	unsigned char* pucMap = (unsigned char*)aLoadTiff.DoIt(0);
	if(pucMap == 0L)
	{	close(iFile);
		return false;
	}
	//-------------------
	for(int y=0; y<m_aiPadSize[1]; y++)
        {       int iSrc = y * iFrmSizeX;
		int iDst = y * m_aiPadSize[0];
		for(int x=0; x<iFrmSizeX; x++)
		{	m_pucBadMap[iDst+x] = m_pucBadMap[iDst+x]
				| (pucMap[iSrc+x] & 0x1);
		}
        }
	close(iFile);
	/*
	Util::CSaveTempMrc aSaveTmpMrc;
        aSaveTmpMrc.SetFile(pInput->m_acTmpFile, "-DefectMap.mrc");
        aSaveTmpMrc.DoIt(pucMap, 0, m_aiFrmSize);
	*/
	if(pucMap != 0L) delete[] pucMap;
        return true;
}

