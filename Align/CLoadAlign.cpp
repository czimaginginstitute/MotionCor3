#include "CAlignInc.h"
#include <memory.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace MotionCor2::Align;

CLoadAlign* CLoadAlign::m_pInstance = 0L;

CLoadAlign* CLoadAlign::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CLoadAlign;
	return m_pInstance;
}

void CLoadAlign::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CLoadAlign::CLoadAlign(void)
{
	m_pFile = 0L;
	m_pFullShift = 0L;
	m_pPatchShifts = 0L;
	m_bLoaded = false;
}

CLoadAlign::~CLoadAlign(void)
{
	this->Clean();
}

void CLoadAlign::Clean(void)
{
	if(m_pFullShift != 0L)
	{	delete m_pFullShift; m_pFullShift = 0L;
	}
	if(m_pPatchShifts != 0L)
	{	delete m_pPatchShifts; m_pPatchShifts = 0L;
	}
	if(m_pFile != 0L) 
	{	fclose(m_pFile); m_pFile = 0L;
	}
	m_bLoaded = false;
}

bool CLoadAlign::Open(char* pcFileName)
{
	this->Clean();
	if(pcFileName == 0L || strlen(pcFileName) == 0) return false;
	//-----------------------------------------------------------
	m_pFile = fopen(pcFileName, "rt");
	m_bLoaded = (m_pFile == 0L) ? false : true;
	if(m_bLoaded) mReadFile();
	//------------------------
	if(m_bLoaded) return true;
	printf( "Info: Alignment file cannot be read.\n\n");
	return false;
}

bool CLoadAlign::Open(char* pcDirName, char* pcStackFile)
{
     this->Clean();
     m_bLoaded = false;
     //----------------
     int iSize = strlen(pcDirName);
     if(pcDirName == 0L || iSize == 0) return false;
     //----------------------------------------------
     char acFileName[256] = {'\0'};
     strcpy(acFileName, pcDirName);
     if(pcDirName[iSize - 1] != '/') strcat(acFileName, "/");
     //------------------------------------------------------
     char* pcSlash = strrchr(pcStackFile, '/');
     char* pcFileName = (pcSlash == 0L) ? pcStackFile : pcSlash+1;
     strcat(acFileName, pcFileName);
     //-----------------------------
     char* pcDot = strrchr(acFileName, '.');
     if(pcDot == 0L) strcat(acFileName, ".aln");
     else strcpy(pcDot, ".aln");
     //-------------------------
     return this->Open(acFileName);
}

bool CLoadAlign::IsLoaded(void)
{
	return m_bLoaded;
}

CStackShift* CLoadAlign::GetFullShift(bool bClean)
{
	CStackShift* pFullShift = m_pFullShift;
	if(bClean) m_pFullShift = 0L;
	return pFullShift;
}

CPatchShifts* CLoadAlign::GetPatchShifts(bool bClean)
{
     CPatchShifts* pPatchShifts = m_pPatchShifts;
     if(bClean) m_pPatchShifts = 0L;
     return pPatchShifts;
}

void CLoadAlign::mReadFile(void)
{
	mReadSetting();
	if(!m_bLoaded)
	{	fclose(m_pFile); m_pFile = 0L; return;
	}
	//--------------------------------------------
	int iNumPatches = m_aiPatches[0] * m_aiPatches[1];
	if(iNumPatches == 0) mReadGlobalShift();
	else mReadPatchShift();
	//---------------------
	fclose(m_pFile);
	m_pFile = 0L;
}

void CLoadAlign::mReadSetting(void)
{
	char acBuf[64] = {'\0'};
	char acLine[256] = {'\0'};
	CSaveAlign* pSaveAlign = CSaveAlign::GetInstance();
	//-------------------------------------------------
	m_bLoaded = false;
	while(!feof(m_pFile))
	{	if(fgets(acLine, 256, m_pFile) == 0L) continue;
		else if(strstr(acLine, pSaveAlign->m_acSetting) != 0L)
		{	m_bLoaded = true; break;
		}
	}
	if(!m_bLoaded) return;
	//--------------------
	int iNumStacks = 0;
	m_bLoaded = false;
	while(!feof(m_pFile))
	{	if(fgets(acLine, 256, m_pFile) == 0L) continue;
		else if(strstr(acLine, pSaveAlign->m_acStackSize) != 0L)
		{	sscanf(acLine, "%s %d %d %d %d", acBuf, 
			   m_aiStkSize+0, m_aiStkSize+1, 
			   m_aiStkSize+2, &iNumStacks);
			m_bLoaded = true; break;
		}
	}
	if(!m_bLoaded) return;
	//--------------------
	m_bLoaded = false;
	while(!feof(m_pFile))
	{	if(fgets(acLine, 256, m_pFile) == 0L) continue;
		else if(strstr(acLine, pSaveAlign->m_acPatches) != 0L)
		{	sscanf(acLine, "%s %d %d %d", acBuf, m_aiPatches+0,
		   	   m_aiPatches+1, m_aiPatches+2);
          		m_bLoaded = true; break;
		}
	}
	if(!m_bLoaded) return;
	//--------------------
	m_bLoaded = false;
	while(!feof(m_pFile))
	{	if(fgets(acLine, 256, m_pFile) == 0L) continue;
     		else if(strstr(acLine, pSaveAlign->m_acThrow) != 0L)
		{	sscanf(acLine, "%s %d %d", acBuf, 
			  m_aiThrow+0, m_aiThrow+1);
			m_bLoaded = true; break;
     		}
	}
	if(!m_bLoaded) return;
	//--------------------
	m_aiStkSize[2] -= (m_aiThrow[0] + m_aiThrow[1]);
}      

void CLoadAlign::mReadGlobalShift(void)
{
	m_pFullShift = new CStackShift;
     	m_pFullShift->Setup(m_aiStkSize[2]);
	//----------------------------------
	char acLine[256] = {'\0'};
	float afShift[2] = {0.0f};
	int iFrame = 0, iCount = 0;
 	//-------------------------
 	while(!feof(m_pFile))
	{	if(fgets(acLine, 256, m_pFile) == 0L) continue;
     		int iItems = sscanf(acLine, "%d  %f  %f", &iFrame, 
		   afShift+0, afShift+1);
		if(iItems != 3) continue;
		m_pFullShift->SetShift(iCount, afShift);
		iCount += 1;
		if(iCount == m_aiStkSize[2]) break;
     	}
	if(iCount != m_aiStkSize[2]) m_bLoaded = false;
	else m_bLoaded = true;
}

void CLoadAlign::mReadPatchShift(void)
{
	mReadGlobalShift();
	if(!m_bLoaded) return;
	//--------------------
	m_pPatchShifts = new CPatchShifts;
	m_pPatchShifts->Setup(m_aiPatches[0], m_aiPatches[1], m_aiStkSize);
	m_pPatchShifts->SetFullShift(m_pFullShift);
	//----------------------------------------- 	
	char acBuf[64] = {'\0'};
	char acLine[256] = {'\0'};
	int iFmCount = 0, iPatCount = 0, iPatchID = 0, iVal = 0;
	float afCenter[2] = {0.0f}, afShift[2] = {0.0f};
	//----------------------------------------------
	CSaveAlign* pSaveAlign = CSaveAlign::GetInstance();
	int iNumPatches = m_aiPatches[0] * m_aiPatches[1];
	CStackShift* pPatShift = 0L;
	//--------------------------
	while(!feof(m_pFile))
	{	if(iFmCount == 0) 
		{	pPatShift = new CStackShift;
			pPatShift->Setup(m_aiStkSize[2]);
		}
		//---------------------------------------
		if(fgets(acLine, 256, m_pFile) == 0L) continue;
		else if(acLine[0] == '#') continue;
		//---------------------------------
		if(strstr(acLine, pSaveAlign->m_acPatchID) != 0L)
          	{	sscanf(acLine, "%s %d", acLine, &iPatchID);
		}
		else if(strstr(acLine, pSaveAlign->m_acConverge) != 0L)
		{	sscanf(acLine, "%s %d\n", acLine, &iVal);
			pPatShift->m_bConverged = (iVal == 0);
		}
		//------------------------------------------------
		int iItems = sscanf(acLine, "%d %f %f %f %f", 
		   &iVal, afCenter+0, afCenter+1, 
		   afShift+0, afShift+1);
		if(iItems != 5) continue;
		pPatShift->SetShift(iFmCount, afShift);
		pPatShift->SetCenter(afCenter[0], afCenter[1]);
		iFmCount += 1;
		//------------
		if(iFmCount == m_aiStkSize[2])
		{	m_pPatchShifts->SetRawShift(pPatShift, iPatCount);
			iFmCount = 0; iPatCount += 1;
			if(iPatCount == iNumPatches) break;
		}
	}
	if(iPatCount != iNumPatches) m_bLoaded = false;
	else m_bLoaded = true;
}

