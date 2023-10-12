#include "CTiffFileInc.h"
#include <cstring>
#include <unistd.h>

using namespace MotionCor2::TiffUtil;

CLoadTiffHeader::CLoadTiffHeader(void)
{
	m_pTiff = 0L;
}


CLoadTiffHeader::~CLoadTiffHeader(void)
{
	if(m_pTiff == 0L) return;
	TIFFCleanup(m_pTiff);
	m_pTiff = 0L;	
}

bool CLoadTiffHeader::DoIt(int iFile)
{
	m_iMode = 0;
	m_fPixelSize = 0.0f;
	memset(m_aiImgSize, 0, sizeof(m_aiImgSize));
	//------------------------------------------
	lseek64(iFile, 0, SEEK_SET);
	m_pTiff = TIFFFdOpen(iFile, "\0", "r");
	if(m_pTiff == 0L) return false;
	//-----------------------------
	m_bReadImgSize = mReadImageSize();
	m_bReadMode = mReadMode();
	//------------------------
	m_bRowsPerStrip = mReadRowsPerStrip();
	m_bTileSize = mReadTileSize();
        //----------------------------
        mReadPixelSize();
   	//---------------
	TIFFCleanup(m_pTiff);
	m_pTiff = 0L;
	//-----------
	if(!m_bReadImgSize) return false;
	if(!m_bReadMode) return false;
	if(!m_bRowsPerStrip && !m_bTileSize) return false;
	return true;
}

int CLoadTiffHeader::GetSizeX(void)
{
	return m_aiImgSize[0];
}

int CLoadTiffHeader::GetSizeY(void)
{
	return m_aiImgSize[1];
}

int CLoadTiffHeader::GetSizeZ(void)
{
	return m_aiImgSize[2];
}

void CLoadTiffHeader::GetSize(int* piSize, int iElems)
{
	if(iElems <= 0) return;
	piSize[0] = m_aiImgSize[0];
	if(iElems > 1) piSize[1] = m_aiImgSize[1];
	if(iElems > 2) piSize[2] = m_aiImgSize[2];
}

int CLoadTiffHeader::GetMode(void)
{
	return m_iMode;
}

float CLoadTiffHeader::GetPixelSize(void)
{
	return m_fPixelSize;
}

int CLoadTiffHeader::GetTileSizeX(void)
{
	return m_aiTileSize[0];
}

int CLoadTiffHeader::GetTileSizeY(void)
{
	return m_aiTileSize[1];
}

int CLoadTiffHeader::GetNumTilesX(void)
{
	return m_aiNumTiles[0];
}

int CLoadTiffHeader::GetNumTilesY(void)
{
	return m_aiNumTiles[1];
}

bool CLoadTiffHeader::IsStrip(void)
{
	return m_bRowsPerStrip;
}

bool CLoadTiffHeader::IsTile(void)
{
	return m_bTileSize;
}

bool CLoadTiffHeader::mReadImageSize(void)
{
	uint32_t uiWidth = 0;
	uint32_t uiLength = 0;
	int iRetX = TIFFGetField(m_pTiff, TIFFTAG_IMAGEWIDTH, &uiWidth);
        int iRetY = TIFFGetField(m_pTiff, TIFFTAG_IMAGELENGTH, &uiLength);
	if(iRetX == 0 || iRetY == 0) return false;
	m_aiImgSize[0] = uiWidth;
	m_aiImgSize[1] = uiLength;
	m_aiImgSize[2] = TIFFNumberOfDirectories(m_pTiff);
	return true;
}

bool CLoadTiffHeader::mReadMode(void)
{
        short sNumBits = 0;
	short sFormat = 0;
	TIFFGetField(m_pTiff, TIFFTAG_BITSPERSAMPLE, &sNumBits);
	bool bFormat = TIFFGetField(m_pTiff, TIFFTAG_SAMPLEFORMAT, &sFormat);
	//-------------------------------------------------------------------
	if(sNumBits == 8)
	{	m_iMode = Mrc::eMrcUChar;
		return true;
	}
        else if(sNumBits == 16)
	{	if(bFormat && sFormat == SAMPLEFORMAT_INT)
		{	m_iMode = Mrc::eMrcShort;
		}
                else
		{	m_iMode = Mrc::eMrcUShort;
		}
		return true;
	}
        else if(bFormat && sFormat == SAMPLEFORMAT_IEEEFP)
        {       m_iMode = Mrc::eMrcFloat;
		return true;
        }
	return false;
}

void CLoadTiffHeader::mReadPixelSize(void)
{
	float fResX = 0.0f;
	float fResY = 0.0f;
	short sResUnit = 0;
	bool bResX = TIFFGetField(m_pTiff, TIFFTAG_XRESOLUTION, &fResX);
	bool bResY = TIFFGetField(m_pTiff, TIFFTAG_YRESOLUTION, &fResY);
	TIFFGetField(m_pTiff, TIFFTAG_RESOLUTIONUNIT, &sResUnit);
	//-------------------------------------------------------
	if(bResX) 
	{	m_fPixelSize = (float)(fResX / 2.54e8);
	}
	else if(bResY)
	{	m_fPixelSize = (float)(fResY / 2.54e8);
	}
	else
	{	m_fPixelSize = 0.0f;
	} 
}

bool CLoadTiffHeader::mReadRowsPerStrip(void)
{
	uint32_t uiRowsPerStrip = 0;
	int iRetVal = TIFFGetField
        (  m_pTiff,
           TIFFTAG_ROWSPERSTRIP,
           &uiRowsPerStrip
        );
	if(iRetVal == 0) return false;
	//----------------------------
	m_aiTileSize[0] = m_aiImgSize[0];
	m_aiTileSize[1] = uiRowsPerStrip;
	m_aiNumTiles[0] = 1;
        m_aiNumTiles[1] = m_aiImgSize[1] / uiRowsPerStrip;
	if((m_aiNumTiles[1] * m_aiTileSize[1]) < m_aiImgSize[1])
	{	m_aiNumTiles[1] += 1;
	}
	return true;
}

bool CLoadTiffHeader::mReadTileSize(void)
{
	uint32_t uiWidth = 0;
	uint32_t uiLength = 0;
	int iRetX = TIFFGetField(m_pTiff, TIFFTAG_TILEWIDTH, &uiWidth);
	int iRetY = TIFFGetField(m_pTiff, TIFFTAG_TILELENGTH, &uiLength);
	if(iRetX == 0 || iRetY == 0) return false;
	//----------------------------------------
	m_aiTileSize[0] = uiWidth;
	m_aiTileSize[1] = uiLength;
	m_aiNumTiles[0] = m_aiImgSize[0] / m_aiTileSize[0];
	m_aiNumTiles[1] = m_aiImgSize[1] / m_aiTileSize[1];
	if((m_aiNumTiles[0] * m_aiTileSize[0]) < m_aiImgSize[0])
	{	m_aiNumTiles[0] += 1;
	}
	if((m_aiNumTiles[1] * m_aiTileSize[1]) < m_aiImgSize[1])
	{	m_aiNumTiles[1] += 1;
	}
	return true;
}

