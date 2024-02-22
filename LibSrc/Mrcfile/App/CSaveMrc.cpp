#include "../Include/CMrcFileInc.h"
#include <Util/Util_FileName.h>
#include <memory.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>

using namespace Mrc;

CSaveMrc::CSaveMrc()
{
	m_iFile = -1;
	m_pSaveMain = new CSaveMainHeader;
	m_pSaveExt = new CSaveExtHeader;
	m_pSaveImg = new CSaveImage;
	//--------------------------
	m_iNumInts = 0;
	m_iNumFloats = 0;
	m_iGainBytes = 0;
}

CSaveMrc::~CSaveMrc()
{
	this->CloseFile();
	if(m_pSaveImg != 0L) delete m_pSaveImg;
	if(m_pSaveExt != 0L) delete m_pSaveExt;
	if(m_pSaveMain != 0L) delete m_pSaveMain;
}

bool CSaveMrc::OpenFile(char* pcFileName, int iSerialNum)
{
	this->CloseFile();
	if(pcFileName == 0L) return false;
	//--------------------------------
	Util_FileName aUtilFileName(pcFileName);
	char* pcSerialFile = aUtilFileName.InsertSerial(iSerialNum);
	bool bOpenFile = this->OpenFile(pcSerialFile);
	if(pcSerialFile != 0L) delete[] pcSerialFile;
	return bOpenFile;
}

bool CSaveMrc::OpenFile(wchar_t* pwcFileName, int iSerialNum)
{
	this->CloseFile();
	if(pwcFileName == 0L) return false;
	int iSize = wcslen(pwcFileName);
	if(iSize == 0) return false;
	//--------------------------
	char* pcFile = new char[iSize + 1];
	for(int i=0; i<iSize; i++) pcFile[i] = (char)pwcFileName[i];
	pcFile[iSize] = 0;
	//----------------
	bool bOpen = this->OpenFile(pcFile, iSerialNum);
	if(pcFile != 0L) delete[] pcFile;
	return bOpen;
}

bool CSaveMrc::OpenFile(char* pcFileName)
{
	this->CloseFile();
	if(pcFileName == 0L) return false;
	//--------------------------------
	mode_t aMode = S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH;
	m_iFile = open(pcFileName, O_RDWR | O_CREAT | O_TRUNC, aMode);
	if(m_iFile == -1) return false;
	//-----------------------------
	m_pSaveMain->SetFile(m_iFile);
	m_pSaveExt->SetFile(m_iFile);
	m_pSaveImg->SetFile(m_iFile);
	return true;
}

bool CSaveMrc::OpenFile(wchar_t* pwcFileName)
{
	if(pwcFileName == 0L) return false;
	if(pwcFileName == 0L) return false;
        int iSize = wcslen(pwcFileName);
        if(iSize == 0) return false;
        //--------------------------
        char* pcFile = new char[iSize + 1];
        for(int i=0; i<iSize; i++) pcFile[i] = (char)pwcFileName[i];
        pcFile[iSize] = 0;
        //----------------
        bool bOpen = this->OpenFile(pcFile);
	if(pcFile != 0L) delete[] pcFile;
	return bOpen;
}

void CSaveMrc::Flush(void)
{
	if(m_iFile == -1) return;
	fsync(m_iFile);
}

void CSaveMrc::CloseFile(void)
{
	if(m_iFile == -1) return;
	//-----------------------
	m_pSaveMain->DoIt();
	m_pSaveExt->DoIt();	
	//-----------------
	m_pSaveMain->Reset();
	m_pSaveMain->SetFile(-1);
	m_pSaveExt->SetFile(-1);
	m_pSaveImg->SetFile(-1);
	close(m_iFile);
	m_iFile = -1;
}

void CSaveMrc::SetMode(int iMode)
{
	m_pSaveMain->SetMode(iMode);
	m_pSaveImg->SetMode(iMode);
}

void CSaveMrc::SetImgSize
(	int* piImgSize, 
	int iNumImgs,
	int iNumImgStacks,
	float fPixelSize
)
{	m_pSaveMain->SetImgSize(piImgSize, iNumImgs, 
	   iNumImgStacks, fPixelSize);
	//----------------------------
	this->SetExtHeader(0, 0, 0);
	m_pSaveImg->SetImgSize(piImgSize);
}

void CSaveMrc::SetExtHeader
(	int iNumInts, 
	int iNumFloats,
	int iGainBytes
)
{	m_iNumInts = iNumInts;
	m_iNumFloats = iNumFloats;
	m_iGainBytes = iGainBytes;
	//------------------------
	int nz = m_pSaveMain->m_aHeader.nz;
	int iSymbt = (iNumInts * sizeof(int) 
	   + iNumFloats * sizeof(float))
	   * nz + iGainBytes;
	m_pSaveMain->SetNumInts(iNumInts);
	m_pSaveMain->SetNumFloats(iNumFloats);
	m_pSaveMain->SetSymbt(iSymbt);
	//----------------------------
	m_pSaveExt->Setup(iNumInts, iNumFloats, nz);
	m_pSaveImg->SetSymbt(iSymbt);
}

void CSaveMrc::DoIt(int iNthImage, void* pvImage)
{
	if(m_iFile == -1) return;
	m_pSaveImg->DoIt(iNthImage, pvImage);
}

void CSaveMrc::DoIt
(	int iStartImg, int iNumImgs, 
	float fMin, float fMax, 
	float fMean, void* pvImages
)
{	if(m_iFile == -1) return;
	m_pSaveMain->SetMinMaxMean(fMin, fMax, fMean);
	m_pSaveImg->DoIt(iStartImg, iNumImgs, pvImages);
}

void CSaveMrc::SaveMinMaxMean(float fMin, float fMax, float fMean)
{
	m_pSaveMain->SetMinMaxMean(fMin, fMax, fMean);
}

void CSaveMrc::DoGain(float* pfGain)
{
	if(pfGain == 0L) return;
	int iGainBytes = m_pSaveMain->GetGainBytes();
	if(iGainBytes == 0) return;
	//-------------------------
	int aiImgSize[3] = {0};
	m_pSaveMain->GetImgSize(aiImgSize, 3);
	int iHdrSize = m_pSaveMain->GetNumInts() * sizeof(int)
		+ m_pSaveMain->GetNumFloats() * sizeof(float);
	int iOffset = 1024 + aiImgSize[2] * iHdrSize;
	m_pSaveExt->SaveGain(iOffset, pfGain, iGainBytes);
}

float CSaveMrc::mCalcMean(void* pvImage)
{
	if(pvImage == 0L) return 0.0f;
	//----------------------------
	int aiImgSize[2];
	m_pSaveMain->GetImgSize(aiImgSize, 2);
	int iMode = m_pSaveMain->GetMode();
	int iPixels = aiImgSize[0] * aiImgSize[1];
	double dSum = 0;
	//--------------
	if(iMode == eMrcUChar || iMode == eMrcUCharEM)
	{	unsigned char* pcImg = (unsigned char*)pvImage;
		for(int i=0; i<iPixels; i++) dSum += pcImg[i];
		return (float)(dSum / iPixels);
	}
	if(iMode == eMrcShort)
	{	short* psImg = (short*)pvImage;
		for(int i=0; i<iPixels; i++) dSum += psImg[i];
		return (float)(dSum / iPixels);
	}
	if(iMode == eMrcUShort)
	{	unsigned short* psImg = (unsigned short*)pvImage;
		for(int i=0; i<iPixels; i++) dSum += psImg[i];
		return (float)(dSum / iPixels);
	}
	if(iMode == eMrcFloat)
	{	float* pfImg = (float*)pvImage;
		for(int i=0; i<iPixels; i++) dSum += pfImg[i];
		return (float)(dSum / iPixels);
	}
	if(iMode == eMrcInt)
	{	int* piImg = (int*)pvImage;
		for(int i=0; i<iPixels; i++) dSum += piImg[i];
		return (float)(dSum / iPixels);
	}
	return 0.0f;
}

float CSaveMrc::mCalcMin(void* pvImage)
{
	if(pvImage == 0L) return 0.0f;
	//----------------------------
	int aiImgSize[2];
	m_pSaveMain->GetImgSize(aiImgSize, 2);
	int iMode = m_pSaveMain->GetMode();
	int iPixels = aiImgSize[0] * aiImgSize[1];
	float fMin = (float)1e20;
	//-----------------------
	if(iMode == eMrcUChar || iMode == eMrcUCharEM)
	{	unsigned char* pcImg = (unsigned char*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMin > pcImg[i]) fMin = pcImg[i];
		}
		return fMin;
	}
	if(iMode == eMrcShort)
	{	short* psImg = (short*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMin > psImg[i]) fMin = psImg[i];
		}
		return fMin;
	}
	if(iMode == eMrcUShort)
	{	unsigned short* psImg = (unsigned short*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMin > psImg[i]) fMin = psImg[i];
		}
		return fMin;
	}
	if(iMode == eMrcFloat)
	{	float* pfImg = (float*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMin > pfImg[i]) fMin = pfImg[i];
		}
		return fMin;
	}
	if(iMode == eMrcInt)
	{	int* piImg = (int*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMin > piImg[i]) fMin = (float)piImg[i];
		}
		return fMin;
	}
	return 0.0f;
}

float CSaveMrc::mCalcMax(void* pvImage)
{
	if(pvImage == 0L) return 0.0f;
	//----------------------------
	int aiImgSize[2];
	m_pSaveMain->GetImgSize(aiImgSize, 2);
	int iMode = m_pSaveMain->GetMode();
	int iPixels = aiImgSize[0] * aiImgSize[1];
	float fMax = (float)-1e20;
	//------------------------
	if(iMode == eMrcUChar || iMode == eMrcUCharEM)
	{	unsigned char* pcImg = (unsigned char*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMax < pcImg[i]) fMax = pcImg[i];
		}
		return fMax;
	}
	if(iMode == eMrcShort)
	{	short* psImg = (short*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMax < psImg[i]) fMax = psImg[i];
		}
		return fMax;
	}
	if(iMode == eMrcUShort)
	{	unsigned short* psImg = (unsigned short*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMax < psImg[i]) fMax = psImg[i];
		}
		return fMax;
	}
	if(iMode == eMrcFloat)
	{	float* pfImg = (float*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMax < pfImg[i]) fMax = pfImg[i];
		}
		return fMax;
	}
	if(iMode == eMrcInt)
	{	int* piImg = (int*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	if(fMax < piImg[i]) fMax = (float)piImg[i];
		}
		return fMax;
	}
	return 0.0f;
}
