#include "CMrcUtilInc.h"
#include <stdio.h>
#include <memory.h>

using namespace MotionCor2::MrcUtil;

CAsyncSingleSave::CAsyncSingleSave(void)
{
	memset(m_acMrcFile, 0, sizeof(m_acMrcFile));
	m_pfImg = 0L;
}

CAsyncSingleSave::~CAsyncSingleSave(void)
{
	if(m_pfImg != 0L) delete[] m_pfImg;
	m_pfImg = 0L;
}

void CAsyncSingleSave::DoIt
(	char* pcMrcFile,
	float* pfImg,
	int* piImgSize,
	float fPixelSize,
	bool bClean,
	bool bAsync
)
{	strcpy(m_acMrcFile, pcMrcFile);
	memcpy(m_aiImgSize, piImgSize, sizeof(m_aiImgSize));
	//--------------------------------------------------
	m_pfImg = pfImg;
	m_fPixelSize = fPixelSize;
	m_bClean = bClean;
	if(bAsync) Util_Thread::Start();
	else mSave();
}

void CAsyncSingleSave::ThreadMain(void)
{
	mSave();
}

void CAsyncSingleSave::mSave(void)
{
	if(m_pfImg == 0L) return;
	//-----------------------
	Mrc::CSaveMrc aSaveMrc;
	if(!aSaveMrc.OpenFile(m_acMrcFile)) 
	{	if(m_bClean) delete[] m_pfImg;
		m_pfImg = 0L;
		return;
	}
	//-------------	
	aSaveMrc.SetMode(Mrc::eMrcFloat);
	aSaveMrc.SetImgSize(m_aiImgSize, 1, 1, m_fPixelSize);
	aSaveMrc.SetExtHeader(0, 0, 0);
	aSaveMrc.DoIt(0, m_pfImg);
	//--------------------------
	if(m_bClean) delete[] m_pfImg;
	m_pfImg = 0L;
}

