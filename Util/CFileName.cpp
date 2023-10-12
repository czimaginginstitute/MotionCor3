#include "CUtilInc.h"
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <ctype.h>
#include <stdio.h>

using namespace MotionCor2::Util;

CFileName::CFileName(void)
{
	mReset();
}

CFileName::~CFileName(void)
{
}

void CFileName::Analyze(char* pcFileName)
{
	mReset();
	if(pcFileName == 0L) return;
	strcpy(m_acFileName, pcFileName);
	//-------------------------------
	mGetMainName();
	mGetSerial();	
}

void CFileName::mReset(void)
{
	m_bMrc = false;
	m_bTiff = false;
	m_iSerial = 0;
	memset(m_acFileName, 0, sizeof(m_acFileName));
	memset(m_acMainName, 0, sizeof(m_acMainName));
}

void CFileName::mGetMainName(void)
{
	char* pcSlash = strrchr(m_acFileName, '/');
	if(pcSlash == 0L) strcpy(m_acMainName, m_acFileName);
	else strcpy(m_acMainName, pcSlash + 1);
	//-------------------------------------
	char* pcMrc = strcasestr(m_acMainName, ".mrc");
	if(pcMrc != 0L)
	{	m_bMrc = true;
		m_bTiff = false;
		return;
	}
	//-------------
	char* pcTif = strcasestr(m_acMainName, ".tif");
	if(pcTif != 0L)
	{	m_bMrc = false;
		m_bTiff = true;
		return;
	}
}

void CFileName::mGetSerial(void)
{
	m_iSerial = 0;
	int iSize = strlen(m_acMainName);
	if(iSize == 0) return;
	//--------------------
	int iFactor = 1;
	for(int i=iSize-1; i>=0; i--)
	{	if(!isdigit(m_acMainName[i]))
		{	if(iFactor > 1) break;
			else continue;
		}
		int iDigit = m_acMainName[i] - '0';
		m_iSerial += (iDigit * iFactor);
		iFactor *= 10;
	}
}
	
	

