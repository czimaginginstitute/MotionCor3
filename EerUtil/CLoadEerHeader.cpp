#include "CEerUtilInc.h"
#include <cstring>
#include <unistd.h>

using namespace MotionCor2::EerUtil;

CLoadEerHeader::CLoadEerHeader(void)
{
	m_aiCamSize[0] = 4096;
	m_aiCamSize[1] = 4096;
	m_iNumFrames = 0;
	m_usCompression = 0;
	m_iNumBits = -1;
	m_iEerSampling = 1;
}


CLoadEerHeader::~CLoadEerHeader(void)
{
}

bool CLoadEerHeader::DoIt(TIFF* pTiff, int iEerSampling)
{
	m_iEerSampling = iEerSampling;
	memset(m_aiCamSize, 0, sizeof(m_aiCamSize));
	memset(m_aiFrmSize, 0, sizeof(m_aiFrmSize));
	//------------------------------------------
	TIFFGetField(pTiff, TIFFTAG_IMAGEWIDTH, &m_aiCamSize[0]);
	TIFFGetField(pTiff, TIFFTAG_IMAGELENGTH, &m_aiCamSize[1]);
	m_iNumFrames = TIFFNumberOfDirectories(pTiff);
	TIFFGetField(pTiff, TIFFTAG_COMPRESSION, &m_usCompression);
	if(m_usCompression == 65000) m_iNumBits = 8;
	else if(m_usCompression == 65001) m_iNumBits = 7;
	else m_iNumBits = -1;
	//-------------------
	bool bHasError = mCheckError();
	if(bHasError) return false;
	//-------------------------
	int iFact = 1;
	if(m_iEerSampling == 2) iFact = 2;
	else if(m_iEerSampling == 3) iFact = 4;
	m_aiFrmSize[0] = m_aiCamSize[0] * iFact;
	m_aiFrmSize[1] = m_aiCamSize[1] * iFact;
	return true;
}

bool CLoadEerHeader::mCheckError(void)
{
	bool bHasError = false;
	const char* pcErrSize = "Error: Invalid image size.";
	const char* pcErrCmp = "Error: Invalid compression.";
	if(m_aiCamSize[0] <= 0 || m_aiCamSize[0] <= 0 || m_iNumFrames <= 0)
	{	fprintf(stderr, "%s %d %d %d\n", pcErrSize,
		   m_aiCamSize[0], m_aiCamSize[1], m_iNumFrames);
		bHasError = true;
	}
	if(m_iNumBits <= 0)
	{	fprintf(stderr, "%s %d\n", pcErrCmp, m_usCompression);
		bHasError = true;
	}
	return bHasError;
}
