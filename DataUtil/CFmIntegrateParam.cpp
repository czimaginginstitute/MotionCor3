#include "CDataUtilInc.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>

using namespace MotionCor2;
using namespace MotionCor2::DataUtil;

CFmIntegrateParam::CFmIntegrateParam(void)
{
	m_piIntFmStart = 0L;
	m_piIntFmSize = 0L;
	m_pfIntFmDose = 0L;
	m_pfAccFmDose = 0L;
	m_pfIntFmCents = 0L;
	m_iNumIntFms = 0;
	m_iMrcMode = -1;
}


CFmIntegrateParam::~CFmIntegrateParam(void)
{
	mClean();
}

void CFmIntegrateParam::Setup(int iNumRawFms, int iMrcMode)
{
	CInput* pInput = CInput::GetInstance();
	iNumRawFms -= (pInput->m_aiThrow[0] + pInput->m_aiThrow[1]);
	if(m_iNumRawFms == iNumRawFms && m_iMrcMode == iMrcMode) return; 
	//--------------------------------------------------------------
	mClean();
	m_iNumRawFms = iNumRawFms;
	m_iMrcMode = iMrcMode;
	//--------------------
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	bool bIntegrate = pReadFmIntFile->NeedIntegrate();
	if(bIntegrate) 
	{	mSetupFileInt();
	}
	else if(pReadFmIntFile->HasDose())
	{	mSetupFile();
	}
	else mSetup();
	//--------------------------
	mCalcIntFmCenters();
	//printf("Raw and rendered frames: %4d  %4d\n\n",
	//   m_iNumRawFms, m_iNumIntFms);

}

int CFmIntegrateParam::GetIntFmStart(int iIntFrame)
{
	return m_piIntFmStart[iIntFrame];
}

int CFmIntegrateParam::GetIntFmSize(int iIntFrame)
{
	return m_piIntFmSize[iIntFrame];
}

int CFmIntegrateParam::GetNumIntFrames(void)
{
	return m_iNumIntFms;
}

float CFmIntegrateParam::GetAccruedDose(int iIntFrame)
{
	return m_pfAccFmDose[iIntFrame];
}

bool CFmIntegrateParam::NeedIntegrate(void)
{
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	return pReadFmIntFile->NeedIntegrate();
}

bool CFmIntegrateParam::DoseWeight(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iKv < 80 || pInput->m_iKv > 300) return false;
	else if(pInput->m_fPixelSize <= 0) return false;
	//----------------------------------------------
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	if(pReadFmIntFile->HasDose()) return true;
	if(pInput->m_fFmDose > 0) return true;
	//------------------------------------
	return false;
}

bool CFmIntegrateParam::DWSelectedSum(void)
{
	if(!DoseWeight()) return false;
	//-----------------------------
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_afSumRange[0] >= pInput->m_afSumRange[1]) return false;
	else return true;
}

bool CFmIntegrateParam::InSumRange(int iIntFrame)
{
	CInput* pInput = CInput::GetInstance();
	if(m_pfAccFmDose[iIntFrame] < pInput->m_afSumRange[0]) return false;
	if(m_pfAccFmDose[iIntFrame] > pInput->m_afSumRange[1]) return false;
	return true;
}

//-------------------------------------------------------------------
// For the case where there is no frame integration file is given.
//-------------------------------------------------------------------
void CFmIntegrateParam::mSetup(void)
{
	CInput* pInput = CInput::GetInstance();
	m_iNumIntFms = m_iNumRawFms; 
	//--------------------------
	mAllocate();
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_piIntFmStart[i] = pInput->m_aiThrow[0] + i;
		m_piIntFmSize[i] = 1;
		//-------------------
		m_pfIntFmDose[i] = pInput->m_fFmDose;
		//-----------------------------------
		int iFmCount = m_piIntFmStart[i] + m_piIntFmSize[i];
		m_pfAccFmDose[i] = iFmCount * pInput->m_fFmDose
		   + pInput->m_fInitDose;
	}
}

void CFmIntegrateParam::mSetupFile(void)
{
	CInput* pInput = CInput::GetInstance();
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	m_iNumIntFms = m_iNumRawFms;
	mAllocate();
	//--------------------------
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_piIntFmStart[i] = pInput->m_aiThrow[0] + i;
		m_piIntFmSize[i] = 1;
	}
	//---------------------------------------------------
	int iNumIntFms = 0;
	for(int i=0; i<pReadFmIntFile->m_iNumEntries; i++)
	{	int iGroupSize = pReadFmIntFile->GetGroupSize(i);
		float fDose = pReadFmIntFile->GetDose(i);
		for(int j=0; j<iGroupSize; j++)
		{	m_pfIntFmDose[iNumIntFms] = fDose;
			iNumIntFms += 1;
			if(iNumIntFms == m_iNumIntFms) break;
		}
		if(iNumIntFms == m_iNumIntFms) break;
	}
	//-------------------------------------------------------
	m_pfAccFmDose[0] = m_pfIntFmDose[0] + pInput->m_fInitDose
	   + pInput->m_aiThrow[0] * pReadFmIntFile->GetDose(0);
	for(int i=1; i<m_iNumIntFms; i++)
	{	m_pfAccFmDose[i] = m_pfIntFmDose[i] + m_pfAccFmDose[i-1];
	}
}

void CFmIntegrateParam::mSetupFileInt(void)
{
	CInput *pInput = CInput::GetInstance();
	CReadFmIntFile* pReadFmIntFile = CReadFmIntFile::GetInstance();
	//------------------------------------------------------------
	// Make sure the total raw frames cannot exceed the stack size
	//------------------------------------------------------------
	int iSumFms = 0;
       	int iLastEntry = pReadFmIntFile->m_iNumEntries - 1;
	int iGpSize0 = pReadFmIntFile->GetGroupSize(0);
	int iGpSizeN = pReadFmIntFile->GetGroupSize(iLastEntry);
	int iNumEntries = pReadFmIntFile->m_iNumEntries;
	//------------------------------------------------------
	for(int i=0; i<iNumEntries; i++)
	{	iSumFms += pReadFmIntFile->GetGroupSize(i);
	}
	int iDiff = iSumFms - m_iNumRawFms;
	iGpSizeN -= iDiff;
	//---------------------------------------------------------
	// calculate for each group (i.e. the line in the frame
	// integration file) number of integrated frames and number
	// of raw frames. make sure they divisible. The extra
	// raw frames will be added to the last integrated frame.
	//---------------------------------------------------------
	int iNumRawFms = 0, iNumIntFms = 0;
	int iLeftRawFms = m_iNumRawFms;
	int* piNumIntFmsPerGroup = new int[iNumEntries];
	int* piNumRawFmsPerGroup = new int[iNumEntries];
	for(int i=0; i<iNumEntries; i++)
	{	int iGrpSize = pReadFmIntFile->GetGroupSize(i);
		int iIntSize = pReadFmIntFile->GetIntSize(i);
		if(iGrpSize > iLeftRawFms) iGrpSize = iLeftRawFms;
		//------------------------------------------------
		piNumIntFmsPerGroup[i] = iGrpSize / iIntSize;
		piNumRawFmsPerGroup[i] = (piNumIntFmsPerGroup[i] * iIntSize);
		iNumIntFms += piNumIntFmsPerGroup[i];
		iNumRawFms += piNumRawFmsPerGroup[i];
		//-----------------------------------
		iLeftRawFms = m_iNumRawFms - iNumRawFms;
		if(iLeftRawFms < iIntSize) break;	
	}
	m_iNumIntFms = iNumIntFms;
	mAllocate();
	//-----------------------------------------------
	// calculate each integrated frame size, ie, the
	// number of raw frames each has.
	//-----------------------------------------------
	iNumIntFms = 0;
	for(int i=0; i<iNumEntries; i++)
	{	int iIntSize = pReadFmIntFile->GetIntSize(i);
		for(int j=0; j<piNumIntFmsPerGroup[i]; j++)
		{	m_piIntFmSize[iNumIntFms] = iIntSize;
			iNumIntFms += 1;
		}
	}
	//--------------------------------------------------------
	// Add the leftover raw frames to the trailing integrated
	// frames, one raw frame for one integrated frame.
	//--------------------------------------------------------
	for(int i=0; i<iLeftRawFms; i++)
	{	int j = m_iNumIntFms - 1 - i;
		m_piIntFmSize[j] += 1;
	}
	m_piIntFmStart[0] = pInput->m_aiThrow[0];
	for(int i=1; i<m_iNumIntFms; i++)
	{	int j = i - 1;
		m_piIntFmStart[i] = m_piIntFmStart[j] + m_piIntFmSize[j];
	}
	//---------------------------------------------------------------
	int iIntSize = pReadFmIntFile->GetIntSize(0);
	float fDose = pReadFmIntFile->GetDose(0);
	if(fDose <= 0) fDose = pInput->m_fFmDose;
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_pfIntFmDose[i] = fDose * m_piIntFmSize[i];
	}
	m_pfAccFmDose[0] = m_pfIntFmDose[0] 
	   + pInput->m_fInitDose
	   + fDose * pInput->m_aiThrow[0];
	for(int i=1; i<m_iNumIntFms; i++)
	{	int j = i - 1;
		m_pfAccFmDose[i] = m_pfIntFmDose[i] + m_pfAccFmDose[j];
	}
	//-------------------------------------------------------------
	delete[] piNumIntFmsPerGroup;
	delete[] piNumRawFmsPerGroup;

	/*
	for(int i=0; i<m_iNumIntFms; i++)
	{	printf("%4d %4d %4d %8.3f %8.3f\n", i, m_piIntFmStart[i],
		   m_piIntFmSize[i], m_pfIntFmDose[i], m_pfAccFmDose[i]);
	}
	*/ 
}

CFmIntegrateParam* CFmIntegrateParam::GetCopy(void)
{
	CFmIntegrateParam* pInst = new CFmIntegrateParam;
	pInst->m_iNumRawFms = m_iNumRawFms;
	pInst->m_iMrcMode = m_iMrcMode;
	pInst->m_iNumIntFms = m_iNumIntFms;
	pInst->mAllocate();
	//-----------------
	int iBytes = sizeof(int) * m_iNumIntFms;
	memcpy(pInst->m_piIntFmStart, m_piIntFmStart, iBytes);
	memcpy(pInst->m_piIntFmSize, m_piIntFmSize, iBytes);
	//--------------------------------------------------
	iBytes = sizeof(float) * m_iNumIntFms * 3;
	memcpy(pInst->m_pfIntFmDose, m_pfIntFmDose, iBytes);
	//--------------------------------------------------
	return pInst;
}

void CFmIntegrateParam::mClean(void)
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

void CFmIntegrateParam::mAllocate(void)
{
	m_piIntFmStart = new int[m_iNumIntFms * 2];
	m_piIntFmSize = &m_piIntFmStart[m_iNumIntFms];
	m_pfIntFmDose = new float[m_iNumIntFms * 3];
	m_pfAccFmDose = &m_pfIntFmDose[m_iNumIntFms];
	m_pfIntFmCents = &m_pfIntFmDose[m_iNumIntFms * 2];
}

void CFmIntegrateParam::mCalcIntFmCenters(void)
{
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_pfIntFmCents[i] = m_piIntFmStart[i] +
		   0.5f * (m_piIntFmSize[i] - 1.0f);
	}
}
