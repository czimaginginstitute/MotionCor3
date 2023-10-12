#include "CDataUtilInc.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>

using namespace MotionCor2;
using namespace MotionCor2::DataUtil;

//=============================================================================
// 1. Raw frames are those read from input movie files (MRC, TIFF, or EER) and 
//    can have different frame exposures/frame dose.
// 2. An integrated frame (IntFm) is the sum of multiple raw frames. An early
//    IntFm may have less raw frames that the later ones.
// 3. IntFmFile is a text file containing multiple entries, one per each line.
//    Each line have three numbers: number of raw frames (groupSize), number 
//    of raw frames to be integrated (intSize), and raw frame's frame dose 
//    (fmDose). number of Integrated frames is groupSize / intSize.
// 4. In each entry each raw frame must have the same frame dose and groupSize
//    should be divisible by intSize.
//=============================================================================  
CEntry::CEntry(void)
{	
	memset(this, 0, sizeof(CEntry));
}

CEntry::~CEntry(void)
{
}

void CEntry::Set(int iGroupSize, int iIntSize, float fFmDose)
{	
	m_iGroupSize = iGroupSize;
	m_iIntSize = iIntSize;
	m_fFmDose = fFmDose;
}

CReadFmIntFile* CReadFmIntFile::m_pInstance = 0L;

CReadFmIntFile* CReadFmIntFile::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CReadFmIntFile;
	return m_pInstance;
}

void CReadFmIntFile::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CReadFmIntFile::CReadFmIntFile(void)
{
	m_ppEntries = 0L;
	m_iNumEntries = 0;
}

CReadFmIntFile::~CReadFmIntFile(void)
{
	mClean();
}

bool CReadFmIntFile::HasDose(void)
{
	if(m_iNumEntries == 0) return false;
	if(m_ppEntries[0]->m_fFmDose <= 0) return false;
	else return true;
}

bool CReadFmIntFile::NeedIntegrate(void)
{
	if(m_iNumEntries == 0) return false;
	//------------------------------------------
	// For variable frame rates in exposure no
	// integration is allowed.
	//-----------------------------------------
	float fDose0 = m_ppEntries[0]->m_fFmDose;
	float fDoseN = m_ppEntries[m_iNumEntries-1]->m_fFmDose;
	float fDif = (float)fabs(fDose0 - fDoseN);
	if(fDif >= 0.0001f) return false;
	//----------------------------------------------------
	// For fixed exposure and users specified integration
	// sizes, integration is then allowed.
	//----------------------------------------------------
	for(int i=0; i<m_iNumEntries; i++)
	{	if(m_ppEntries[i]->m_iIntSize > 1) return true;
	}
	return false;
}

int CReadFmIntFile::GetGroupSize(int iEntry)
{
	return m_ppEntries[iEntry]->m_iGroupSize;
}

int CReadFmIntFile::GetIntSize(int iEntry)
{
	return m_ppEntries[iEntry]->m_iIntSize;
}

float CReadFmIntFile::GetDose(int iEntry)
{
	return m_ppEntries[iEntry]->m_fFmDose;
}

void CReadFmIntFile::DoIt(void)
{
	mClean();
	//-------
	CInput* pInput = CInput::GetInstance();
	FILE* pFile = fopen(pInput->m_acFmIntFile, "rt");
	if(pFile == 0L) return;
	//------------------------
	int iGroupSize = 0, iIntSize = 0;
	float fFmDose = 0.0f;
	std::queue<CEntry*> entryQueue;
	//---------------------------------------------------
	// The frame integration file must contain 3 columns.
	//---------------------------------------------------
	char acBuf[256] = {'\0'};
	fgets(acBuf, 256, pFile);
	int iItems = sscanf(acBuf, "%d  %d  %f",
	   &iGroupSize, &iIntSize, &fFmDose);
	bool bSuccess = (iItems >= 2) ? true : false;
	bool bTwoItems = (iItems == 2) ? true : false;
	if(bTwoItems) fFmDose = pInput->m_fFmDose;
	//----------------------------------------
	while(bSuccess)
	{	CEntry* pEntry = new CEntry;
		pEntry->Set(iGroupSize, iIntSize, fFmDose);
		entryQueue.push(pEntry);
		if(feof(pFile)) break;
		//--------------------
		char* pcRet = fgets(acBuf, 256, pFile);
		if(pcRet == 0L) break;
		//--------------------
		if(bTwoItems) 
		{	iItems = sscanf(acBuf, "%d  %d", 
			   &iGroupSize, &iIntSize);
			if(iItems != 2) continue;
		}
		else
		{	iItems == sscanf(acBuf, "%d  %d  %f",
			   &iGroupSize, &iIntSize, &fFmDose);
			if(iItems != 3) continue;
			else if(fFmDose < 0) bSuccess = false;
		}
	}
	fclose(pFile);
	//------------
	m_iNumEntries = entryQueue.size();
	if(m_iNumEntries == 0) return;
	else m_ppEntries = new CEntry*[m_iNumEntries];
	//---------------------------------------------
	bool bSameDose = true;
	for(int i=0; i<m_iNumEntries; i++)
	{	m_ppEntries[i] = entryQueue.front();
		entryQueue.pop();
		if(m_ppEntries[0]->m_fFmDose != m_ppEntries[i]->m_fFmDose)
		{	bSameDose = false;
		}
	}
	//-----------------------------------------------------------
	// Each entry must have the same frame dose. Otherwise, this
	// movie is collected with various frame exposure, no frame
	// integration will be performed.
	//-----------------------------------------------------------
	if(!bSameDose) mClean();
}

void CReadFmIntFile::mClean(void)
{
	if(m_ppEntries == 0L) return;
	for(int i=0; i<m_iNumEntries; i++)
	{	delete m_ppEntries[i];
	}
	delete[] m_ppEntries;
	m_ppEntries = 0L;
	m_iNumEntries = 0;
}
