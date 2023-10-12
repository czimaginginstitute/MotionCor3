#include "CUtilInc.h"
#include <stdio.h>

using namespace MotionCor2::Util;

CNextItem::CNextItem(void)
{
	pthread_mutex_init(&m_aMutex, 0L);
}

CNextItem::~CNextItem(void)
{
	pthread_mutex_destroy(&m_aMutex);
}

void CNextItem::Create(int iNumItems)
{
	m_iNumItems = iNumItems;
	m_iNextItem = 0;
}

void CNextItem::Reset(void)
{
     m_iNextItem = 0;
}

int CNextItem::GetNext(void)
{
	if(m_iNumItems <= 0) return -1;
	//-----------------------------
	int iNext = -1;
	pthread_mutex_lock(&m_aMutex);
	if(m_iNextItem < m_iNumItems)
	{	iNext = m_iNextItem;
		m_iNextItem++;
	}
	pthread_mutex_unlock(&m_aMutex);
	return iNext;
}

int CNextItem::GetNumItems(void)
{
	return m_iNumItems;
}

