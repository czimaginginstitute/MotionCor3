#include "CDataUtilInc.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <string.h>
#include <memory.h>

using namespace MotionCor2::DataUtil;


CAlnSums::CAlnSums(void)
{
}

CAlnSums::~CAlnSums(void)
{
}

void CAlnSums::Setup(bool bDoseWeight, bool bDoseSelected)
{
	int iSum = 0;
	strcpy(m_acExts[iSum], "");
	//-----------------
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iAlign == 0)
	{	if(pInput->m_iSplitSum == 0) return;
		strcpy(m_acExts[1], "_ODD");
		strcpy(m_acExts[2], "_EVN");
		return;
	}
	//-----------------
	iSum += 1;
	if(bDoseWeight)
	{	strcpy(m_acExts[iSum], "_DW");
		iSum += 1;
		if(bDoseSelected) 
		{	strcpy(m_acExts[iSum], "_DWS");
			iSum += 1;
		}
	}
	//------------------------
	strcpy(m_acExts[iSum], "_ODD");
	strcpy(m_acExts[iSum+1], "_EVN");
}

void CAlnSums::GetFileExt(int iIndex, char* pcExt)
{
	strcpy(pcExt, m_acExts[iIndex]);
}
