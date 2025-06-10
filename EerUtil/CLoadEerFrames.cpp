#include "CEerUtilInc.h"
#include "../CMainInc.h"
#include <tiffio.h>
#include <memory.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
using namespace MotionCor2::EerUtil;

CLoadEerFrames::CLoadEerFrames(void)
{
	m_pucFrames = 0L;
	m_puiFrmStarts = 0L;
	m_puiFrmSizes = 0L;
}

CLoadEerFrames::~CLoadEerFrames(void)
{
	this->Clean();
}

void CLoadEerFrames::Clean(void)
{
	if(m_pucFrames != 0L) delete[] m_pucFrames;
	m_pucFrames = 0L;
	if(m_puiFrmStarts != 0L) delete[] m_puiFrmStarts;
	m_puiFrmStarts = 0L;
	if(m_puiFrmSizes != 0L) delete[] m_puiFrmSizes;
	m_puiFrmSizes = 0L;
}

bool CLoadEerFrames::DoIt
(	int iFile,
	TIFF* pTiff, 
	CLoadEerHeader* pLoadHeader
)
{	this->Clean();
	m_pTiff = pTiff;
	m_iNumFrames = pLoadHeader->m_iNumFrames;
	//---------------------------------------
	struct stat buf;
	fstat(iFile, &buf);
	unsigned int uiSize = buf.st_size;
	m_pucFrames = new unsigned char[uiSize];
	memset(m_pucFrames, 0, sizeof(char) * uiSize);
	//--------------------------------------------
	int iBytes = sizeof(int) * m_iNumFrames;
	m_puiFrmStarts = new unsigned int[m_iNumFrames];
	m_puiFrmSizes = new unsigned int[m_iNumFrames];
	memset(m_puiFrmStarts, 0, iBytes);
	memset(m_puiFrmSizes, 0, iBytes);
	//------------------------------
	m_uiBytesRead = 0;
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iTiffOrder >= 0)
	{	for(int i=0; i<m_iNumFrames; i++) mReadFrame(i);
	}
	else
	{	int iLastFrm = m_iNumFrames - 1;
		for(int i=iLastFrm; i>=0; i--) mReadFrame(i);
	}
	m_pTiff = 0L;
	return true;
}

unsigned char* CLoadEerFrames::GetEerFrame(int iFrame)
{
	if(m_pucFrames == 0L) return 0L;
	return m_pucFrames + m_puiFrmStarts[iFrame];
}

int CLoadEerFrames::GetEerFrameSize(int iFrame)
{
	if(m_puiFrmSizes == 0L) return 0;
	return (int)m_puiFrmSizes[iFrame];
}

void CLoadEerFrames::mReadFrame(int iFrame)
{
	m_puiFrmStarts[iFrame] = m_uiBytesRead;
	unsigned char* pcEerFrm = m_pucFrames + m_uiBytesRead;
	//---------------------------------------------------
	TIFFSetDirectory(m_pTiff, iFrame);
	int iNumStrips = TIFFNumberOfStrips(m_pTiff);
	unsigned int uiFrmBytes = 0;
	//----------------
	for(int i=0; i<iNumStrips; i++)
	{	int iStripBytes = TIFFRawStripSize(m_pTiff, i);
		TIFFReadRawStrip(m_pTiff, i, pcEerFrm + uiFrmBytes, 
		   iStripBytes);
		uiFrmBytes += iStripBytes;
	}
	m_puiFrmSizes[iFrame] = uiFrmBytes;
	m_uiBytesRead += uiFrmBytes;
}

