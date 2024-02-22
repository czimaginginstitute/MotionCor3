#include "CDataUtilInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>

using namespace MotionCor2;
using namespace MotionCor2::DataUtil;

CFmGroupParam::CFmGroupParam(void)
{
	m_iBinZ = 1;
	m_iNumGroups = 0;
	m_iNumIntFms = 0;
	m_piGroupStart = 0L;
	m_piGroupSize = 0L;
	m_pfGroupCenters = 0L;
}


CFmGroupParam::~CFmGroupParam(void)
{
	this->mClean();
}

void CFmGroupParam::Setup(int iBinZ, CFmIntParam* pFmIntParam)
{
	if(m_iNumIntFms == pFmIntParam->m_iNumIntFms 
	   && m_iBinZ == iBinZ) return;
	else this->mClean();
	//-----------------------------
	m_iNumIntFms = pFmIntParam->m_iNumIntFms;
	m_iNumGroups = m_iNumIntFms;
	m_iBinZ = iBinZ;
	m_bGrouping = (m_iBinZ > 1) ? true : false;	
	//-----------------------------------------
	mAllocate();
	//---------------------------------------------------------------
	// When a movie is collected with variable frame rate, the frame
	// integration file should specify 1 for its int size (2nd col).
	// In this case, the IntFmSize should be 1 and we group based on
	// the dose.
	// Example of group by size. Example of group by dose
	// 20   2   0.5              20  1  0.5
	// 40   4   0.5              40  1  0.7
	// 80   8   0.5              80  1  1.2
	//---------------------------------------------------------------
	int iIntFmSize0 = pFmIntParam->GetIntFmSize(0);
	float fIntFmDose0 = pFmIntParam->m_pfIntFmDose[0];
	bool bGroupByDose = (fIntFmDose0 < 0.001) ? false : true;
	for(int i=1; i<m_iNumIntFms; i++)
	{	int iIntFmSize = pFmIntParam->GetIntFmSize(i);
		int fIntFmDose = pFmIntParam->m_pfIntFmDose[i];
		if(iIntFmSize != iIntFmSize0 || fIntFmDose < 0.001) 
		{	bGroupByDose = false; break;
		}
	}
	//------------------------------------------
	if(bGroupByDose) mGroupByDose(pFmIntParam);
	else mGroupByRawSize(pFmIntParam);
	//------------------------------------------------------------
	// Calculate number of raw frames in each group. Note that
	// each group contains certain numbers of integrated frames
	// that are sums of certain numbers of raw frames.
	//------------------------------------------------------------
	int iNumRawFms = 0;
	for(int g=0; g<m_iNumGroups; g++)
	{	int iGroupRawFms = 0;
		for(int i=0; i<m_piGroupSize[g]; i++)
		{	int iIntFm = m_piGroupStart[g] + i;
			iGroupRawFms += pFmIntParam->GetIntFmSize(iIntFm);
		}
		m_pfGroupCenters[g] = iNumRawFms + 0.5f * (iGroupRawFms - 1);
		iNumRawFms += iGroupRawFms;
	}	
	/*	
	for(int i=0; i<m_iNumGroups; i++)
	{	printf("%3d  %3d  %3d  %3d, %8.2f\n", i, m_piGroupStart[i],
		   m_piGroupSize[i], m_piGroupStart[i] + m_piGroupSize[i],
		   m_pfGroupCenters[i]);
	}
	*/
}

void CFmGroupParam::mGroupByRawSize(CFmIntParam* pFmIntParam)
{
	m_iNumGroups = 0;
	int iIntFm = 0;
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_piGroupStart[i] = iIntFm;
		int iRawFms = 0;
		while(true)
		{	iRawFms += pFmIntParam->GetIntFmSize(iIntFm);
			m_piGroupSize[i] += 1;
			iIntFm += 1;
			if(iIntFm >= m_iNumIntFms) break;
			if(iRawFms >= m_iBinZ) break;
		}
		m_iNumGroups += 1;
		if(iIntFm >= m_iNumIntFms) break;
	}
}

void CFmGroupParam::mGroupByDose(CFmIntParam* pFmIntParam)
{
	float fMinDose = pFmIntParam->m_pfIntFmDose[0];
	for(int i=1; i<m_iNumIntFms; i++)
	{	float fDose = pFmIntParam->m_pfIntFmDose[i];
		if(fDose < fMinDose) fMinDose = fDose;
	}
	float fGroupDose = m_iBinZ * fMinDose;
	float fTolDose = 0.01f * fGroupDose;
	//----------------------------------
	m_iNumGroups = 0;
	int iIntFmCount = 0;
	for(int i=0; i<m_iNumIntFms; i++)
	{	m_piGroupStart[i] = iIntFmCount;
		float fDoseSum = 0.0f;
		while(true)
		{	fDoseSum += pFmIntParam->m_pfIntFmDose[iIntFmCount];
			m_piGroupSize[i] += 1;
			iIntFmCount += 1;
			//---------------
			if(iIntFmCount >= m_iNumIntFms) break;
			float fDifDose = fDoseSum - fGroupDose;
			if(fDifDose > fTolDose) break;
			else if((-fDifDose) < fTolDose) break;
		}
		m_iNumGroups += 1;
		if(iIntFmCount >= m_iNumIntFms) break;			
	}
}

int CFmGroupParam::GetGroupStart(int iGroup)
{
	return m_piGroupStart[iGroup];
}

int CFmGroupParam::GetGroupSize(int iGroup)
{
	return m_piGroupSize[iGroup];
}

float CFmGroupParam::GetGroupCenter(int iGroup)
{
	return m_pfGroupCenters[iGroup];
}

CFmGroupParam* CFmGroupParam::GetCopy(void)
{
	CFmGroupParam* pInst = new CFmGroupParam;
	pInst->m_iBinZ = m_iBinZ;
	pInst->m_bGrouping = m_bGrouping;
	pInst->m_iNumIntFms = m_iNumIntFms;
	pInst->m_iNumGroups = m_iNumGroups;
	//---------------------------------
	pInst->mAllocate();
	int iBytes = sizeof(int) * m_iNumIntFms;
	memcpy(pInst->m_piGroupStart, m_piGroupStart, iBytes);
	memcpy(pInst->m_piGroupSize, m_piGroupSize, iBytes);
	//--------------------------------------------------
	iBytes = sizeof(float) * m_iNumIntFms;
	memcpy(pInst->m_pfGroupCenters, m_pfGroupCenters, iBytes);
	//--------------------------------------------------------
	return pInst;
}

void CFmGroupParam::mAllocate(void)
{
	m_piGroupStart = new int[m_iNumIntFms * 2];
	m_piGroupSize = m_piGroupStart + m_iNumIntFms;
	memset(m_piGroupStart, 0, sizeof(int) * m_iNumIntFms * 2);
	//--------------------------------------------------------
	m_pfGroupCenters = new float[m_iNumIntFms];
	memset(m_pfGroupCenters, 0, sizeof(float) * m_iNumIntFms);
}

void CFmGroupParam::mClean(void)
{
	if(m_piGroupStart != 0L) delete[] m_piGroupStart;
	m_piGroupStart = 0L;
	m_piGroupSize = 0L;
	m_iNumIntFms = 0;
	m_iNumGroups = 0;
}
