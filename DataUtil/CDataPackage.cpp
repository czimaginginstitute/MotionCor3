#include "CDataUtilInc.h"
#include "../CMainInc.h"
#include "../FindCtf/CFindCtfInc.h"
#include <stdio.h>
#include <string.h>
#include <memory.h>

using namespace MotionCor2;
using namespace MotionCor2::DataUtil;

CDataPackage::CDataPackage(void)
{
	m_pcInFileName = 0L;
	m_pcSerial = 0L;
	m_pRawStack = 0L;
	m_pAlnStack = 0L;
	m_pAlnSums = 0L;
	m_pCtfStack = 0L;
	m_pvCtfParam = 0L;
	m_pFmIntParam = 0L;
	m_pFmGroupParams = 0L;
}

CDataPackage::~CDataPackage(void)
{
	this->DeleteAll();
}

void CDataPackage::SetInputFileName(char* pcFileName)
{
	if(m_pcInFileName != 0L) 
	{	delete[] m_pcInFileName;
	}
	m_pcInFileName = pcFileName;
}

char* CDataPackage::GetInputFileName(bool bClean)
{
	char* pcInFileName = m_pcInFileName;
	if(bClean) m_pcInFileName = 0L;
	return pcInFileName;
}

void CDataPackage::SetSerial(char* pcSerial)
{
	if(m_pcSerial != 0L) delete[] m_pcSerial;
	m_pcSerial = pcSerial;
}

char* CDataPackage::GetSerial(bool bClean)
{
	char* pcSerial = m_pcSerial;
	if(bClean) m_pcSerial = 0L;
	return pcSerial;
}

void CDataPackage::SetRawStack(CMrcStack* pRawStack)
{
	if(m_pRawStack != 0L) delete m_pRawStack;
	m_pRawStack = pRawStack;
}

CMrcStack* CDataPackage::GetRawStack(bool bClean)
{
	CMrcStack* pRawStack = m_pRawStack;
	if(bClean) m_pRawStack = 0L;
	return pRawStack;
}

void CDataPackage::SetAlnStack(CMrcStack* pAlnStack)
{
	if(m_pAlnStack != 0L) delete m_pAlnStack;
	m_pAlnStack = pAlnStack;
}

CMrcStack* CDataPackage::GetAlnStack(bool bClean)
{
	CMrcStack* pAlnStack = m_pAlnStack;
	if(bClean) m_pAlnStack = 0L;
	return pAlnStack;
}

void CDataPackage::SetAlnSums(CAlnSums* pAlnSums)
{
	if(m_pAlnSums != 0L) delete m_pAlnSums;
	m_pAlnSums = pAlnSums;
}

CMrcStack* CDataPackage::GetAlnSums(bool bClean)
{
	CMrcStack* pAlnSums = m_pAlnSums;
	if(bClean) m_pAlnSums = 0L;
	return pAlnSums;
}

void CDataPackage::SetCtfStack(CMrcStack* pCtfStack)
{
	if(m_pCtfStack != 0L) delete m_pCtfStack;
	m_pCtfStack = pCtfStack;
}

CMrcStack* CDataPackage::GetCtfStack(bool bClean)
{
	if(!bClean) return m_pCtfStack;
	CMrcStack* pRetStack = m_pCtfStack;
	m_pCtfStack = 0L;
	return pRetStack;
}

void CDataPackage::SetCtfParam(void* pvCtfParam)
{
	if(m_pvCtfParam == 0L) 
	{	m_pvCtfParam = pvCtfParam;
		return;
	}
	delete (FindCtf::CCtfParam*)m_pvCtfParam;
	m_pvCtfParam = pvCtfParam;
}

void* CDataPackage::GetCtfParam(bool bClean)
{
	void* pvCtfParam = m_pvCtfParam;
	if(bClean) m_pvCtfParam = 0L;
	return pvCtfParam;
}

void CDataPackage::DeleteInputFile(void)
{
	if(m_pcInFileName == 0L) return;
	delete[] m_pcInFileName; m_pcInFileName = 0L;
}

void CDataPackage::DeleteSerial(void)
{
	if(m_pcSerial == 0L) return;
	delete[] m_pcSerial; m_pcSerial = 0L;
}

void CDataPackage::DeleteRawStack(void)
{	
	if(m_pRawStack == 0L) return;
	delete m_pRawStack; m_pRawStack = 0L;
}

void CDataPackage::DeleteAlnStack(void)
{
	if(m_pAlnStack == 0L) return;
	delete m_pAlnStack; m_pAlnStack = 0L;
}

void CDataPackage::DeleteAlnSums(void)
{
	if(m_pAlnSums == 0L) return;
	delete m_pAlnSums; m_pAlnSums = 0L;
}

void CDataPackage::DeleteCtfStack(void)
{
	if(m_pCtfStack == 0L) return;
	delete m_pCtfStack; m_pCtfStack = 0L;
}

void CDataPackage::DeleteCtfParam(void)
{
	if(m_pvCtfParam == 0L) return;
	delete (FindCtf::CCtfParam*)m_pvCtfParam;
	m_pvCtfParam = 0L;
}

void CDataPackage::DeleteFmIntParam(void)
{
	if(m_pFmIntParam == 0L) return;
	delete m_pFmIntParam; 
	m_pFmIntParam = 0L;
}

void CDataPackage::DeleteFmGroupParams(void)
{
	if(m_pFmGroupParams == 0L) return;
	delete[] m_pFmGroupParams;
	m_pFmGroupParams = 0L;
}

void CDataPackage::DeleteAll(void)
{
	this->DeleteInputFile();
	this->DeleteSerial();
	this->DeleteRawStack();
	this->DeleteAlnStack();
	this->DeleteAlnSums();
	this->DeleteCtfStack();
	this->DeleteCtfParam();
	this->DeleteFmIntParam();
	this->DeleteFmGroupParams();
}
	
