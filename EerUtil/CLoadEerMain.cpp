#include "CEerUtilInc.h"
#include "../CMainInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <Util/Util_Time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/sysinfo.h>
#include <string.h>
#include <stdio.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::EerUtil;

static int s_aiStkSize[3] = {0};
static int s_iFile = -1;
static TIFF* s_pTiff = 0L;
static CLoadEerMain* s_pLoadEerMain = 0L;
//-------------------------------------------------
static DataUtil::CStackFolder* s_pStackFolder = 0L;
static DataUtil::CDataPackage* s_pPackage = 0L;
static DataUtil::CFmIntParam* s_pFmIntParam = 0L;

static void sCloseFile(void)
{
        if(s_pTiff != 0L) TIFFCleanup(s_pTiff); 
        if(s_iFile != -1) close(s_iFile);
	s_pTiff = 0L;
	s_iFile = -1;
}

static void sCleanPackage(void)
{	if(s_pPackage == 0L) return;
	delete s_pPackage;
	s_pPackage = 0L;
}

CLoadEerMain::CLoadEerMain(void)
{
}

CLoadEerMain::~CLoadEerMain(void)
{
}

bool CLoadEerMain::OpenFile(int aiStkSize[3])
{
	sCloseFile();
	sCleanPackage();
	//--------------
	bool bPop = true, bClean = true;
	s_pStackFolder = DataUtil::CStackFolder::GetInstance();
	s_pPackage = s_pStackFolder->GetPackage(bPop);
	//-----------------
	s_iFile = open(s_pPackage->m_pcInFileName, O_RDONLY);
	if(s_iFile == -1)
	{	fprintf(stderr, "Error: Unable to open EER file, skip.\n"
		   "   %s\n\n", s_pPackage->m_pcInFileName);
		sCleanPackage(); 
		return false;
	}
	//-----------------
	lseek64(s_iFile, 0, SEEK_SET);
	s_pTiff = TIFFFdOpen(s_iFile, "\0", "r");
	if(s_pTiff == 0L) return false;
	//-----------------
	CInput* pInput = CInput::GetInstance();
	CLoadEerHeader aLoadEerHeader;
	bool bSuccess = aLoadEerHeader.DoIt(s_pTiff, pInput->m_iEerSampling);
	if(!bSuccess)
	{	sCloseFile();
		sCleanPackage();
		return false;
	}
	//------------------------------------------------------------------
	// If s_pPackage does not have m_pFmIntParam, this means it is a
	// new package and we need to create one.
	//------------------------------------------------------------------
	if(s_pPackage->m_pFmIntParam == 0L)
	{	s_pPackage->m_pFmIntParam = new DataUtil::CFmIntParam;
	}
	s_pPackage->m_pFmIntParam->Setup
	( aLoadEerHeader.m_iNumFrames, Mrc::eMrcUChar
	);
	delete s_pPackage->m_pFmIntParam;
	s_pPackage->m_pFmIntParam = new DataUtil::CFmIntParam;
	s_pPackage->m_pFmIntParam->Setup
        ( aLoadEerHeader.m_iNumFrames, Mrc::eMrcUChar
        );
	//----------------------------------------------------------------
	// Need two group parameters, one for global, one for local align.
	//----------------------------------------------------------------
	if(s_pPackage->m_pFmGroupParams == 0L)
	{	s_pPackage->m_pFmGroupParams = new DataUtil::CFmGroupParam[2];
	}
	s_pPackage->m_pFmGroupParams[0].Setup(pInput->m_aiGroup[0],
	   s_pPackage->m_pFmIntParam);
	s_pPackage->m_pFmGroupParams[1].Setup(pInput->m_aiGroup[1],
	   s_pPackage->m_pFmIntParam);
	//---------------------------------------------------------------
	// Need s_aiStkSize to allocate CPU/GPU buffer in CBufferPool.
	//---------------------------------------------------------------
	memcpy(s_aiStkSize, aLoadEerHeader.m_aiFrmSize, sizeof(int) * 2);
	s_aiStkSize[2] = s_pPackage->m_pFmIntParam->GetNumIntFrames();
	//-----------------
	printf("EER stack: %d  %d  %d\nRendered stack: %d  %d  %d\n\n", 
	   aLoadEerHeader.m_aiCamSize[0], aLoadEerHeader.m_aiCamSize[1], 
	   aLoadEerHeader.m_iNumFrames, s_aiStkSize[0],
	   s_aiStkSize[1], s_aiStkSize[2]);
	memcpy(aiStkSize, s_aiStkSize, sizeof(int) * 3);
	return true;
}

//-------------------------------------------------------------------
void* CLoadEerMain::GetPackage(void)
{
	if(s_pPackage == 0L) return 0L;
	if(s_pLoadEerMain == 0L) return 0L;
	//---------------------------------
	s_pLoadEerMain->WaitForExit(-1.0f);
	bool bLoaded = s_pLoadEerMain->m_bLoaded;
	delete s_pLoadEerMain; s_pLoadEerMain = 0L;
	sCloseFile();
	//-----------
	if(!bLoaded) sCleanPackage();
	DataUtil::CDataPackage* pRetPackage = s_pPackage;
	s_pPackage = 0L;
	return pRetPackage;
}

void CLoadEerMain::AsyncLoad(void)
{	
	//-----------------------------------------------------------
	// If the load package does not have the raw stack (in this
	// case, the rendered stack), create one.
	//-----------------------------------------------------------
	if(s_pPackage == 0L) return;
	if(s_pPackage->m_pRawStack == 0L)
	{	s_pPackage->m_pRawStack = new DataUtil::CMrcStack;
	}
	s_pPackage->m_pRawStack->Create(Mrc::eMrcUChar, s_aiStkSize);
	//-----------------------------------------------------------
	s_pLoadEerMain = new CLoadEerMain;
	s_pLoadEerMain->Start();
}

void CLoadEerMain::ThreadMain(void)
{
	CInput* pInput = CInput::GetInstance();
        CLoadEerHeader aLoadEerHeader;
        aLoadEerHeader.DoIt(s_pTiff, pInput->m_iEerSampling);
	//---------------------------------------------------
	CLoadEerFrames aLoadEerFrames;
	aLoadEerFrames.DoIt(s_iFile, s_pTiff, &aLoadEerHeader);
	//-----------------------------------------------------
	CRenderMrcStack aRenderMrcStack;
	aRenderMrcStack.DoIt(&aLoadEerHeader, &aLoadEerFrames, s_pPackage);
	m_bLoaded = true;
}

