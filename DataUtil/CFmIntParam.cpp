#include "CDataUtilInc.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>

using namespace MotionCor2;
using namespace MotionCor2::DataUtil;

CFmIntParam::CFmIntParam(void)
{
	m_piIntFmStart = 0L;
	m_piIntFmSize = 0L;
	m_pfIntFmDose = 0L;
	m_pfAccFmDose = 0L;
	m_pfIntFmCents = 0L;
	m_iNumIntFms = 0;
	m_iMrcMode = -1;
}


CFmIntParam::~CFmIntParam(void)
{
	mClean();
}

void CFmIntParam::Setup(int iNumRawFms, int iMrcMode)
{
	CInput* pInput = CInput::GetInstance();
	if(m_iNumRawFms == iNumRawFms && m_iMrcMode == iMrcMode) return; 
	//-----------------
	mClean();
	m_iNumRawFms = iNumRawFms;
	m_iMrcMode = iMrcMode;
	//-----------------
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	bool bIntegrate = pReadFmIntFile->NeedIntegrate();
	if(bIntegrate) mCalcIntFms(); 
	else mSetup();
	//-----------------
	mCalcIntFmCenters();
}

int CFmIntParam::GetIntFmStart(int iIntFrame)
{
	return m_piIntFmStart[iIntFrame];
}

int CFmIntParam::GetIntFmSize(int iIntFrame)
{
	return m_piIntFmSize[iIntFrame];
}

int CFmIntParam::GetNumIntFrames(void)
{
	return m_iNumIntFms;
}

float CFmIntParam::GetAccruedDose(int iIntFrame)
{
	return m_pfAccFmDose[iIntFrame];
}

float CFmIntParam::GetTotalDose(void)
{
	return m_pfAccFmDose[m_iNumIntFms - 1];
}

bool CFmIntParam::bIntegrate(void)
{
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	return pReadFmIntFile->NeedIntegrate();
}

//--------------------------------------------------------------------
// Note: Dose weighting is done in AreTomo, not in MotionCor.
//--------------------------------------------------------------------
bool CFmIntParam::bDoseWeight(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iKv < 80 || pInput->m_iKv > 300) return false;
	else if(pInput->m_fPixelSize <= 0) return false;
	//-----------------
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	if(pReadFmIntFile->HasDose()) return true;
	if(pInput->m_fFmDose > 0) return true;
	//-----------------
	return false;
}

bool CFmIntParam::bDWSelectedSum(void)
{
	if(!bDoseWeight()) return false;
	//-----------------
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_afSumRange[0] < 0) return false;
	if(pInput->m_afSumRange[1] <= 0) return false;
	//-----------------
	float fDif = pInput->m_afSumRange[1] - pInput->m_afSumRange[0];
	if(fDif <= 1.0f) return false;
	//-----------------
	return true;
}

bool CFmIntParam::bInSumRange(int iIntFrame)
{
        CInput* pInput = CInput::GetInstance();
        if(m_pfAccFmDose[iIntFrame] < pInput->m_afSumRange[0]) return false;
        if(m_pfAccFmDose[iIntFrame] > pInput->m_afSumRange[1]) return false;
        return true;
}

//-------------------------------------------------------------------
// For the case where there is no frame integration file is given.
//-------------------------------------------------------------------
void CFmIntParam::mSetup(void)
{
	CInput* pInput = CInput::GetInstance();
	int iNumThrows = pInput->m_aiThrow[0] + pInput->m_aiThrow[1];
	m_iNumIntFms = m_iNumRawFms - iNumThrows;
	//-----------------
	mAllocate();
	//-----------------
	float fFmDose = pInput->m_fFmDose;
	//-----------------
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_piIntFmStart[i] = i + pInput->m_aiThrow[0];
		m_piIntFmSize[i] = 1;
		m_pfIntFmDose[i] = fFmDose;
		//----------------
		int iFmCount = m_piIntFmStart[i] + m_piIntFmSize[i];
		m_pfAccFmDose[i] = fFmDose * (i+1);
	}
}

void CFmIntParam::mCalcIntFms(void)
{
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
        int iNumEntries = pReadFmIntFile->m_iNumEntries;
	int iCount = 0, iExtra = 0, iIntSize = 0;
	//-----------------------------------------------
	// 1) Check whether the number frames in the
	// FmIntFile matches the m_iNumRawFms.
	// 2) If more, subtract extra from the each
	// entry from last to first.
	// 3) If less, add extra to the last entry
	//-----------------------------------------------
	int* piGroupSizes = new int[iNumEntries * 2];
	int* piNumInts = &piGroupSizes[iNumEntries];
	//-----------------
	for(int i=0; i<iNumEntries; i++)
	{	piGroupSizes[i] = pReadFmIntFile->GetGroupSize(i);
		iCount += pReadFmIntFile->GetGroupSize(i);
	}
	//-----------------
	iExtra = iCount - m_iNumRawFms;
	int iLastEntry = iNumEntries - 1;
	if(iExtra > 0)
	{	for(int i=iLastEntry; i>=0; i--)
		{	int iGroupSize = piGroupSizes[i];
			piGroupSizes[i] -= iExtra;
			if(piGroupSizes[i] <= 0) piGroupSizes[i] = 0;
			else iExtra = iExtra - iGroupSize;
			//---------------
			if(iExtra <= 0) break;
		}
	}
	else if(iExtra < 0) piGroupSizes[iNumEntries-1] -= iExtra;
	//-----------------------------------------------
	// 1) For each entry the extra frames are moved
	// to the next entry. As a result, only the
	// last entry may have extra raw frames.
	//-----------------------------------------------
	for(int i=0; i<iLastEntry; i++)
	{	iIntSize = pReadFmIntFile->GetIntSize(i);
		piNumInts[i] = piGroupSizes[i] / iIntSize;
		piGroupSizes[i+1] += (piGroupSizes[i] % iIntSize);
	}
	//-----------------
	iIntSize = pReadFmIntFile->GetIntSize(iLastEntry);
	iExtra = piGroupSizes[iLastEntry] % iIntSize;
	piNumInts[iLastEntry] = piGroupSizes[iLastEntry] / iIntSize;
	//-----------------------------------------------
	// 1) Count total integrated frames.
	// 2) distribute the extra raw frames to the
	// last integrated frames one by one.
	//-----------------------------------------------
	m_iNumIntFms = 0;
	for(int i=0; i<iNumEntries; i++)
	{	m_iNumIntFms += piNumInts[i];
	}
	mAllocate();
	//-----------------
	iCount = 0;
	for(int i=0; i<iNumEntries; i++)
	{	iIntSize = pReadFmIntFile->GetIntSize(i);
		for(int k=0; k<piNumInts[i]; k++)
		{	m_piIntFmSize[iCount] = iIntSize;
			iCount += 1;
		}
	}
	if(piGroupSizes != 0L) delete[] piGroupSizes;
	//-----------------
	for(int i=0; i<iExtra; i++)
	{	int j = m_iNumIntFms - 1 - i;
		m_piIntFmSize[j] += 1;
	}
	//-----------------
	m_piIntFmStart[0] = 0;
	for(int i=1; i<m_iNumIntFms; i++)
	{	int k = i - 1;
		m_piIntFmStart[i] = m_piIntFmStart[k] + m_piIntFmSize[k];
	}
	//-----------------
	float fRawFmDose = pReadFmIntFile->GetDose(0);
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_pfIntFmDose[i] = m_piIntFmSize[i] * fRawFmDose;
	}
	m_pfAccFmDose[0] = m_pfIntFmDose[0];
	for(int i=1; i<m_iNumIntFms; i++)
	{	m_pfAccFmDose[i] = m_pfAccFmDose[i-1] + m_pfIntFmDose[i];
	}
	//------------------
	mThrowIntFrames();
	mCalcIntFmCenters();
	//mDisplay();
}

void CFmIntParam::mThrowIntFrames(void)
{	
	CInput* pInput = CInput::GetInstance();
	int iNumThrows = pInput->m_aiThrow[0] + pInput->m_aiThrow[1];
	if(iNumThrows <= 0) return;
	//-----------------
	int iReducedFms = m_iNumIntFms - iNumThrows;
	if(iReducedFms <= 5)
	{	printf("Warning: throwing two many frames, ignore!\n"
		   "   throw first %d and last %d frames, only\n"
		   "   %d frames left\n\n", pInput->m_aiThrow[0],
		   pInput->m_aiThrow[1], iReducedFms);
		return;
	}
	//-----------------
	m_iNumIntFms = iReducedFms;
	if(pInput->m_aiThrow[0] == 0) return;
	//-----------------
	for(int i=0; i<iReducedFms; i++)
	{	int j = pInput->m_aiThrow[0] + i;
		m_piIntFmStart[i] = m_piIntFmStart[j];
		m_piIntFmSize[i] = m_piIntFmSize[j];
		m_pfIntFmDose[i] = m_pfIntFmDose[j];
		m_pfAccFmDose[i] = m_pfAccFmDose[j];
	}
}


void CFmIntParam::mClean(void)
{
	if(m_piIntFmStart != 0L) delete[] m_piIntFmStart;
	if(m_pfIntFmDose != 0L) delete[] m_pfIntFmDose;
	m_piIntFmStart = 0L;
	m_piIntFmSize = 0L;
	m_pfIntFmDose = 0L;
	m_pfAccFmDose = 0L;
	m_pfIntFmCents = 0L;
	m_iNumRawFms = 0;
	m_iNumIntFms = 0;
}

void CFmIntParam::mAllocate(void)
{
	m_piIntFmStart = new int[m_iNumIntFms * 2];
	m_piIntFmSize = &m_piIntFmStart[m_iNumIntFms];
	m_pfIntFmDose = new float[m_iNumIntFms * 3];
	m_pfAccFmDose = &m_pfIntFmDose[m_iNumIntFms];
	m_pfIntFmCents = &m_pfIntFmDose[m_iNumIntFms * 2];
}

void CFmIntParam::mCalcIntFmCenters(void)
{
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_pfIntFmCents[i] = m_piIntFmStart[i] +
		   0.5f * (m_piIntFmSize[i] - 1.0f);
	}
}

//--------------------------------------------------------------------
// 1. This is for debugging and usually commented out.
//--------------------------------------------------------------------
void CFmIntParam::mDisplay(void)
{
	printf("\n Calculation of frame integration\n");
	printf(" IntFm  Start  Size    Dose     AccDose\n");
	for(int i=0; i<m_iNumIntFms; i++)
	{	printf("%4d   %4d  %5d  %8.3f  %9.3f\n", i, 
		   m_piIntFmStart[i], m_piIntFmSize[i], 
		   m_pfIntFmDose[i], m_pfAccFmDose[i]);
	}
	printf("\n");
}
