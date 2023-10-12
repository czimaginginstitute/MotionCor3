#include "CTiffFileInc.h"
#include "tiffio.h"
#include <memory.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
using namespace MotionCor2::TiffUtil;

CLoadTiffImage::CLoadTiffImage(void)
{
	m_pTiff = 0L;
	m_iImgBytes = 0;
    	memset(m_aiSize, 0, sizeof(m_aiSize));
}

CLoadTiffImage::~CLoadTiffImage(void)
{
	if(m_pTiff == 0L) return;
	TIFFCleanup((TIFF*)m_pTiff);
        m_pTiff = 0L;
}

bool CLoadTiffImage::SetFile(int iFile)
{
	bool bLoadHeader = m_aLoadHeader.DoIt(iFile);
	if(!bLoadHeader) return false;
	//----------------------------
	m_iMode = m_aLoadHeader.GetMode();
	if(m_iMode == Mrc::eMrcUChar) m_iPixelBytes = 1;
	else if(m_iMode == Mrc::eMrcUCharEM) m_iPixelBytes = 1;
	else if(m_iMode == Mrc::eMrcShort) m_iPixelBytes = 2;
	else if(m_iMode == Mrc::eMrcUShort) m_iPixelBytes= 2;
	else if(m_iMode == Mrc::eMrcFloat) m_iPixelBytes = 4;
	else return false;
	//----------------
    	m_aLoadHeader.GetSize(m_aiSize, 3);
    	m_iImgBytes = m_aiSize[0] * m_aiSize[1] * m_iPixelBytes;
	//------------------------------------------------------
    	lseek64(iFile, 0, SEEK_SET);
    	m_pTiff = TIFFFdOpen(iFile, "\0", "r");
	return true;
}

void* CLoadTiffImage::DoIt(int iNthImage)
{
	if(m_pTiff == 0L) return 0L;
	//--------------------------
	int iBytes = m_aiSize[0] * m_aiSize[1] * m_iPixelBytes;
	if(iBytes <= 0) return 0L;
	//------------------------
    	void* pvImage = new char[iBytes];
    	DoIt(iNthImage, pvImage);
	return pvImage;
}

bool CLoadTiffImage::DoIt(int iNthImage, void* pvImage)
{
	bool bSuccess = false;
	if(m_pTiff == 0L) return bSuccess;
    	if(pvImage == 0L) return bSuccess;
    	if(iNthImage < 0 || iNthImage >= m_aiSize[2]) return bSuccess;
	if(m_aLoadHeader.IsStrip())  
	{	bSuccess = mReadByStrip(iNthImage, pvImage);
	}
	else 
	{	bSuccess = mReadByTile(iNthImage, pvImage);
	}
	return bSuccess;
}

bool CLoadTiffImage::mReadByStrip
(	int iNthImage, 
	void* pvImage
) 
{	TIFFSetDirectory(m_pTiff, iNthImage);
	//-----------------------------------
	int iRowBytes = m_aiSize[0] * m_iPixelBytes;
	int iRowsPerStrip = m_aLoadHeader.GetTileSizeY();
	int iNumStrips = m_aLoadHeader.GetNumTilesY();
	int iStripBytes = iRowsPerStrip * iRowBytes;
	char* pcBuf = new char[iStripBytes];
	char* pcImg = (char*)pvImage;
	//---------------------------
	bool bSuccess = true;
    	for(int i=0; i<iNumStrips; i++)
	{	int iStartY = m_aiSize[1] - (i+1) * iRowsPerStrip;
		int iEndY = m_aiSize[1] - 1 - i * iRowsPerStrip;
		if(iStartY < 0) 
		{	iStartY = 0;
			if(iStartY > iEndY) break;
		}
		tsize_t tBytes = TIFFReadEncodedStrip(m_pTiff, i, 
		   pcBuf, (tsize_t)-1);
		if(tBytes == -1) 
		{	bSuccess = false;
			fprintf(stderr, "error: load tiff image %d, %d strip\n",
			   iNthImage, i);
			break;
		}
		//---------------------------------------------------
		for(int y=iStartY; y<=iEndY; y++)
		{	int iRow = iEndY - y;
			char* pcSrc = pcBuf + iRow * iRowBytes;
			char* pcDst = pcImg + y * iRowBytes;
			memcpy(pcDst, pcSrc, iRowBytes);
		}
	}
	if(pcBuf != 0L) delete[] pcBuf;
	return bSuccess;
}

bool CLoadTiffImage::mReadByTile
(	int iNthImage,
	void* pvImage
)
{	TIFFSetDirectory(m_pTiff, iNthImage);
	//-----------------------------------
	int iNumTilesX = m_aLoadHeader.GetNumTilesX();
	int iNumTilesY = m_aLoadHeader.GetNumTilesY();
	int iTileSizeX = m_aLoadHeader.GetTileSizeX();
	int iTileSizeY = m_aLoadHeader.GetTileSizeY();
	int iTileBytes = iTileSizeX * iTileSizeY * m_iPixelBytes;
	char* pcBuf = new char[iTileBytes];
	char* pcImg = (char*)pvImage;
	//---------------------------
	for(int iTileY=0; iTileY<iNumTilesY; iTileY++)
	{	int iStartY = m_aiSize[1] - (iTileY+1) * iTileSizeY;
		int iEndY = m_aiSize[1] - iTileY * iTileSizeY - 1;
		if(iStartY < 0) 
		{	iStartY = 0;
			if(iStartY > iEndY) break;
		}
		//--------------------------------
		for(int iTileX=0; iTileX<iNumTilesX; iTileX++)
		{	int iStartX = iTileX * iTileSizeX;
			int iEndX = iStartX + iTileSizeX - 1;
			if(iEndX >= m_aiSize[0]) 
			{	iEndX = m_aiSize[0] - 1;
				if(iStartX > iEndX) break;
			}
			//--------------------------------
			int iTile = iTileY * iNumTilesX + iTileX;
			TIFFReadEncodedTile
			(  m_pTiff, iTile, pcBuf, iTileBytes
			);
			//----------------------------------
			int iLineBytes = (iEndX - iStartX + 1) 
				* m_iPixelBytes;
			for(int y=iStartY; y<=iEndY; y++)
			{	char* pcSrc = pcBuf + (iEndY - y)
					* iTileSizeX;
				char* pcDst = pcImg + y * m_aiSize[0]
					+ iStartX;
				memcpy(pcDst, pcSrc, iLineBytes);
			}
		}
	}
	if(pcBuf != 0L) delete[] pcBuf;
	return true;
}

