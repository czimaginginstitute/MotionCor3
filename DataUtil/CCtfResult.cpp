#include "CFindCtfInc.h"
#include "../CInput.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::DataUtil;

CCtfResult::CCtfResult(void)
{
	mInit();
	m_iNumCols = 7;
}

//------------------------------------------------------------------------------
// m_pSpectStack: It is CMrcStack that stores the diagnostic power spectrum
//    images that are embedded with the estimated CTF functions.
// m_iNumImgs: It is 1 for single-particle micrograph and the number of tilt
//    images in a tilt series.
// Note: we can use m_iNumImgs to determine if this is a single micrograph
//    or a tilt series.
//------------------------------------------------------------------------------
void CCtfResult::mInit(void)
{
	m_iNumImgs = 0;
	m_pfDfMins = 0L;
	m_pSpectStack = 0L;
}

CCtfResult::~CCtfResult(void)
{
	this->Clean();
}

void CCtfResult::Setup(int iNumImgs, int* piSpectSize)
{
	this->Clean();
	m_iNumImgs = iNumImgs;
	//---------------------------
	m_pfDfMins = new float[m_iNumImgs * m_iNumCols];
	m_pfDfMaxs = &m_pfDfMins[m_iNumImgs];
	m_pfAzimuths = &m_pfDfMaxs[m_iNumImgs];
	m_pfExtPhases = &m_pfAzimuths[m_iNumImgs];
	m_pfScores = &m_pfExtPhases[m_iNumImgs]; 
	m_pfTilts = &m_pfScores[m_iNumImgs];
	m_pfCtfReses = &m_pfTilts[m_iNumImgs];
	//---------------------------
	int iBytes = sizeof(float) * m_iNumImgs * m_iNumCols;
	memset(m_pfDfMins, 0, iBytes);
	//----------------------------
	int aiStkSize[] = {piSpectSize[0], piSpectSize[1], m_iNumImgs};
	m_pSpectStack = new CMrcStack;
	m_pSpectStack->Create(2, aiStkSize);
}

void CCtfResult::Clean(void)
{
	if(m_iNumImgs == 0) return;
	//-------------------------
	if(m_pfDfMins != 0L) delete[] m_pfDfMins;
	if(m_pSpectStack != 0L) delete m_pSpectStack;
	//-------------------------------------------
	mInit();
}

void CCtfResult::SetTilt(int iImage, float fTilt)
{
	m_pfTilts[iImage] = fTilt;
}

void CCtfResult::SetDfMin(int iImage, float fDfMin)
{
	m_pfDfMins[iImage] = fDfMin;
}

void CCtfResult::SetDfMax(int iImage, float fDfMax)
{
	m_pfDfMaxs[iImage] = fDfMax;
}

void CCtfResult::SetAzimuth(int iImage, float fAzimuth)
{
	m_pfAzimuths[iImage] = fAzimuth;
}

void CCtfResult::SetExtPhase(int iImage, float fExtPhase)
{
	m_pfExtPhases[iImage] = fExtPhase;
}

void CCtfResult::SetScore(int iImage, float fScore)
{
	m_pfScores[iImage] = fScore;
}

void CCtfResult::SetCtfScore(int iImage, float fCtfScore)
{
	m_pfCtfReses[iImage] = fCtfScore;
}

void CCtfResult::SetSpect(int iImage, float* pfSpect)
{
	if(m_ppfSpects[iImage] != 0L) delete[] m_ppfSpects[iImage];
	m_ppfSpects[iImage] = pfSpect;
}

float CCtfResult::GetTilt(int iImage)
{
	return m_pfTilts[iImage];
}

float CCtfResult::GetDfMin(int iImage)
{
	return m_pfDfMins[iImage];
}

float CCtfResult::GetDfMax(int iImage)
{
	return m_pfDfMaxs[iImage];
}

float CCtfResult::GetAzimuth(int iImage)
{
	return m_pfAzimuths[iImage];
}

float CCtfResult::GetExtPhase(int iImage)
{
	return m_pfExtPhases[iImage];
}

float CCtfResult::GetScore(int iImage)
{
	return m_pfScores[iImage];
}

float CCtfResult::GetCtfRes(int iImage)
{
	return m_pfCtfReses[iImage];
}

float* CCtfResult::GetSpect(int iImage, bool bClean)
{
	float* pfSpect = m_ppfSpects[iImage];
	if(bClean) m_ppfSpects[iImage] = 0L;
	return pfSpect;
}

void CCtfResult::SaveImod(void)
{
	CInput* pInput = CInput::GetInstance();
	Util::CFileName aInMrcFile, aOutMrcFile;
	aInMrcFile.Setup(pInput->m_acInMrcFile);
	aOutMrcFile.Setup(pInput->m_acOutMrcFile);
	char acFileName[256] = {'\0'}, acBuf[256] = {'\0'};
	aOutMrcFile.GetFolder(acFileName);
	aInMrcFile.GetName(acBuf);
	strcat(acFileName, acBuf);
	strcat(acFileName, "_CTF.txt");	
	//-----------------------------
	FILE* pFile = fopen(acFileName, "w");
	if(pFile == 0L) return;
	//---------------------
	float fExtPhase = this->GetExtPhase(0);
	if(fExtPhase == 0) fprintf(pFile, "1  0  0.0  0.0  0.0  3\n");
	else fprintf(pFile, "5  0  0.0  0.0  0.0  3\n");
	//----------------------------------------------
	const char *pcFormat1 = "%4d  %4d  %7.2f  %7.2f  %8.2f  "
	   "%8.2f  %7.2f\n";
	const char *pcFormat2 = "%4d  %4d  %7.2f  %7.2f  %8.2f  "
	   "%8.2f  %7.2f  %8.2f\n";
	float fDfMin, fDfMax;
	if(fExtPhase == 0)
	{	for(int i=0; i<m_iNumImgs; i++)
		{	float fTilt = this->GetTilt(i);
			fDfMin = this->GetDfMin(i) * 0.1f;
			fDfMax = this->GetDfMax(i) * 0.1f;
			fprintf(pFile, pcFormat1, i+1, i+1, fTilt, fTilt,
			   fDfMin, fDfMax, this->GetAzimuth(i));
		}
	}
	else
	{	for(int i=0; i<m_iNumImgs; i++)
		{	float fTilt = this->GetDfMin(i);
			fDfMin = this->GetDfMin(i) * 0.1f;
			fDfMax = this->GetDfMax(i) * 0.1f;
			fprintf(pFile, pcFormat2, i+1, i+1, fTilt, fTilt,
			   fDfMin, fDfMax, this->GetAzimuth(i),
			   this->GetExtPhase(i));
		}
	}
	fclose(pFile);
}

void CCtfResult::Display(int iNthCtf)
{
	printf("%4d  %8.2f  %8.2f  %6.2f %6.2f %9.5f %6.2\n", iNthCtf+1,
	   this->GetDfMin(iNthCtf), this->GetDfMax(iNthCtf),
           this->GetAzimuth(iNthCtf), this->GetExtPhase(iNthCtf),
           this->GetScore(iNthCtf), this->GetCtfRes(iNthCtf));
}

void CCtfResult::DisplayAll(void)
{
	for(int i=0; i<m_iNumImgs; i++) this->Display(i);
}
