#include "CEerUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::EerUtil;

CDecodeEerFrame::CDecodeEerFrame(void)
{
}

CDecodeEerFrame::~CDecodeEerFrame(void)
{
}

void CDecodeEerFrame::Setup(int* piCamSize, int iEerUpSampling)
{
	m_aiCamSize[0] = piCamSize[0];
	m_aiCamSize[1] = piCamSize[1];
	m_uiCamPixels = piCamSize[0] * piCamSize[1];
	m_iUpSampling = iEerUpSampling;
	//-----------------------------
	int iFact = 1;
	if(m_iUpSampling == 2) 
	{	iFact = 2;
		m_aiSuperResAnd[0] = 2;
		m_aiSuperResAnd[1] = 8;
		m_aiSuperResShift[0] = 1;
		m_aiSuperResShift[1] = 1;
		m_aiSuperResShift[2] = 3;
	}
	else if(m_iUpSampling == 3) 
	{	iFact = 4;
		m_aiSuperResAnd[0] = 3;
		m_aiSuperResAnd[1] = 12;
		m_aiSuperResShift[0] = 2;
		m_aiSuperResShift[1] = 0;
		m_aiSuperResShift[2] = 2;
	}
	m_aiFrmSize[0] = piCamSize[0] * iFact;
	m_aiFrmSize[1] = piCamSize[1] * iFact;
}

void CDecodeEerFrame::Do7Bits
(	unsigned char* pucEerFrame,
	int iEerFrameSize,
	unsigned char* pucRawFrame
)
{	m_pucEerFrame = pucEerFrame;
	m_iEerFrameSize = iEerFrameSize;
	m_pucRawFrame = pucRawFrame;
	//--------------------------
	if(m_iUpSampling == 1) mDo7BitsCounted();
	else mDo7BitsSuperRes();
}

void CDecodeEerFrame::Do8Bits
(	unsigned char* pucEerFrame,
	int iEerFrameSize,
	unsigned char* pucRawFrame
)
{	m_pucEerFrame = pucEerFrame;
	m_iEerFrameSize = iEerFrameSize;
	m_pucRawFrame = pucRawFrame;
	//--------------------------
	if(m_iUpSampling == 1) mDo8BitsCounted();
	else mDo8BitsSuperRes();
}

void CDecodeEerFrame::mDo7BitsCounted(void)
{
	m_uiNumPixels = 0;
	unsigned int uiBitPos = 0;
	unsigned char p;
	unsigned int uiChunk = 0;
	//-----------------------
	while(true)
	{	unsigned int uiFirstByte = uiBitPos >> 3;
		unsigned int uiBitOffset = uiBitPos & 7;  // % 8
		uiChunk = (*(unsigned int*)(m_pucEerFrame + uiFirstByte))
		   >> uiBitOffset;
		//----------------
		p = (unsigned char)(uiChunk & 127);
		uiBitPos += 7;
		m_uiNumPixels += p;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		else if(p == 127) continue;
		//-------------------------
		uiBitPos += 4;
		m_uiX = m_uiNumPixels % m_aiFrmSize[0];
		m_uiY = m_uiNumPixels / m_aiFrmSize[0];
		m_pucRawFrame[m_uiY * m_aiFrmSize[0] + m_uiX] += 1;
		m_uiNumPixels += 1;
		//-----------------
		p = (unsigned char)((uiChunk >> 11) & 127);
		uiBitPos += 7;
		m_uiNumPixels += p;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		else if(p == 127) continue;
		//-------------------------
		uiBitPos += 4;
		m_uiX = m_uiNumPixels % m_aiFrmSize[0];
		m_uiY = m_uiNumPixels / m_aiFrmSize[0];
		m_pucRawFrame[m_uiY * m_aiFrmSize[0] + m_uiX] += 1;
		m_uiNumPixels += 1;
	}
}

void CDecodeEerFrame::mDo7BitsSuperRes(void)
{
	m_uiNumPixels = 0;
	unsigned int uiBitPos = 0;
	unsigned char p;
	unsigned int uiChunk = 0;
	//-----------------------
	while(true)
	{	unsigned int uiFirstByte = uiBitPos >> 3;
		unsigned int uiBitOffset = uiBitPos & 7; // % 8
		uiChunk = (*(unsigned int*)(m_pucEerFrame + uiFirstByte))
		   >> uiBitOffset;
		//----------------
		p = (unsigned char)(uiChunk & 127);
		uiBitPos += 7;
		m_uiNumPixels += p;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		else if(p == 127) continue;
		//-------------------------
		m_ucS = (unsigned char)((uiChunk >> 7) & 15) ^ 0x0A;
		uiBitPos += 4;
		//------------
		mFindElectron();
		m_uiNumPixels += 1;
		//-----------------
		p = (unsigned char)((uiChunk >> 11) & 127);
		uiBitPos += 7;
		m_uiNumPixels += p;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		else if(p == 127) continue;
		//-------------------------
		m_ucS = (unsigned char)((uiChunk >> 18) & 15) ^ 0x0A;
		uiBitPos += 4;
		mFindElectron();
		m_uiNumPixels += 1;
	}
}

void CDecodeEerFrame::mDo8BitsCounted(void)
{
	m_uiNumPixels = 0;
	unsigned int uiPos = 0;
	unsigned char p1, p2;
	while(uiPos < m_iEerFrameSize)
	{	p1 = m_pucEerFrame[uiPos];
		m_uiNumPixels += p1;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		//-------------------------------------
		if(p1 < 255)
		{	m_uiX = m_uiNumPixels % m_aiFrmSize[0];
			m_uiY = m_uiNumPixels / m_aiFrmSize[0];
			m_pucRawFrame[m_uiY * m_aiFrmSize[0] + m_uiX] += 1;
			m_uiNumPixels += 1;
		}
		//-------------------------
		p2 = (m_pucEerFrame[uiPos + 1] >> 4) 
		   | (m_pucEerFrame[uiPos + 2] << 4); 
		m_uiNumPixels += p2;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		//-------------------------------------
		if(p2 < 255)
		{	m_uiX = m_uiNumPixels % m_aiFrmSize[0];
			m_uiY = m_uiNumPixels / m_aiFrmSize[0];
			m_pucRawFrame[m_uiY * m_aiFrmSize[0] + m_uiX] += 1;
			m_uiNumPixels += 1;
		}
		uiPos += 3;
	}	
}

void CDecodeEerFrame::mDo8BitsSuperRes(void)
{
	m_uiNumPixels = 0;
	unsigned int uiPos = 0, uiPos1 = 0, uiPos2 = 0;
	unsigned char p1, p2;
	while(uiPos < m_iEerFrameSize)
	{	uiPos1 = uiPos + 1;
		p1 = m_pucEerFrame[uiPos];
		m_ucS = m_pucEerFrame[uiPos1] & 0x0A;
		m_uiNumPixels += p1;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		if(p1 < 255)
		{	mFindElectron();
			m_uiNumPixels++;
		}
		//----------------------
		p2 = (m_pucEerFrame[uiPos1] >> 4) | (m_pucEerFrame[uiPos2]);
		m_ucS = (m_pucEerFrame[uiPos2] >> 4) ^ 0x0A;
		m_uiNumPixels += p2;
		if(m_uiNumPixels >= m_uiCamPixels) break;
		if(p2 < 255)
		{	mFindElectron();
			m_uiNumPixels++;
		}
		uiPos += 3;
	}
}

void  CDecodeEerFrame::mFindElectron(void)
{
	m_uiX = ((m_uiNumPixels & 4095) << m_aiSuperResShift[0])
	   | ((m_ucS & m_aiSuperResAnd[0]) >> m_aiSuperResShift[1]);
	m_uiY = ((m_uiNumPixels >> 12) << m_aiSuperResShift[0])
	   | ((m_ucS & m_aiSuperResAnd[1]) >> m_aiSuperResShift[2]);
	m_pucRawFrame[m_uiY * m_aiFrmSize[0] + m_uiX] += 1;
}
