#include "CUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <math.h>

using namespace MotionCor2::Util;

CGroupFrames::CGroupFrames(void)
{
	m_piGroupStarts = 0L;
	m_piGroupSizes = 0L;
	m_iNumGroups = 0;
	m_iGroupSize = 0;
}

CGroupFrames::~CGroupFrames(void)
{
	this->mClean();
}

void CGroupFrames::DoGroupSize
(	int iNumFrames, // total frames in raw stack
	int iGroupSize // number of frames per group
)
{	mClean();
	//-------
	m_iNumFrames = iNumFrames;
	m_iGroupSize = iGroupSize;
	if(m_iGroupSize < 1) m_iGroupSize = 1;
	else if(m_iGroupSize > m_iNumFrames) m_iGroupSize = m_iNumFrames;
	//---------------------------------------------------------------
	m_iNumGroups = m_iNumFrames / m_iGroupSize;
	mGroup();
}

void CGroupFrames::DoNumGroups
(	int iNumFrames,
	int iNumGroups
)
{	int iGroupSize = iNumFrames / iNumGroups;
	if(iGroupSize < 1) iGroupSize = 1;
	this->DoGroupSize(iNumFrames, iGroupSize);
}

int CGroupFrames::GetGroupStart(int iGroup)
{
	return m_piGroupStarts[iGroup];
}

int CGroupFrames::GetGroupSize(int iGroup)
{
	return m_piGroupSizes[iGroup];
}

int CGroupFrames::GetNumGroups(void)
{
	return m_iNumGroups;
}

int CGroupFrames::GetNumFrames(void)
{
	return m_iNumFrames;
}

int CGroupFrames::GetGroup(int iFrame)
{
	int iGroup = iFrame / m_iGroupSize;
	//---------------------------------
	int iStart = m_piGroupStarts[iGroup];
	int iEnd = iStart + m_piGroupSizes[iGroup] - 1;
	if(iFrame < iStart) iGroup -= 1;
	else if(iFrame > iEnd) iGroup += 1;
	//---------------------------------
	if(iGroup < 0) iGroup = 0;
	else if(iGroup >= m_iNumGroups) iGroup = m_iNumGroups - 1;
	//--------------------------------------------------------
	return iGroup;
}

void CGroupFrames::mClean(void)
{
	if(m_piGroupStarts != 0L) delete[] m_piGroupStarts;
	if(m_piGroupSizes != 0L) delete[] m_piGroupSizes;
	m_piGroupStarts = 0L;
	m_piGroupSizes = 0L;
	m_iNumGroups = 0;
}

void CGroupFrames::mGroup(void)
{
	int iLeft = m_iNumFrames % m_iGroupSize;
	//--------------------------------------
	m_piGroupSizes = new int[m_iNumGroups];
        for(int i=0; i<m_iNumGroups; i++)
        {       m_piGroupSizes[i] = m_iGroupSize;
        }
        for(int i=0; i<iLeft; i++)
        {       int j = m_iNumGroups - 1 - i;
                m_piGroupSizes[j] += 1;
        }
	//-----------------------------
	m_piGroupStarts = new int[m_iNumGroups];
        m_piGroupStarts[0] = 0;
        for(int i=1; i<m_iNumGroups; i++)
        {       m_piGroupStarts[i] = m_piGroupStarts[i-1]
                  + m_piGroupSizes[i-1];
        }	
}
