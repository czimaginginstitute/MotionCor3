#include "CFindCtfInc.h"
#include "../CInput.h"
#include <stdio.h>
#include <memory.h>

using namespace MotionCor2::FindCtf;

CSaveCtfResults* CSaveCtfResults::m_pInstance = 0L;
static float s_fD2R = 0.017453f;

CSaveCtfResults* CSaveCtfResults::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CSaveCtfResults;
	return m_pInstance;
}

void CSaveCtfResults::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CSaveCtfResults::CSaveCtfResults(void)
{
}

CSaveCtfResults::~CSaveCtfResults(void)
{
}

void CSaveCtfResults::AsyncSave(void)
{	
	this->WaitForExit(-1.0f);
	this->Start();
}

void CSaveCtfResults::ThreadMain(void)
{
	CInput* pInput = CInput::GetInstance();
	Util::CFileName aInputName(pInput->m_acInMrcFile);
	Util::CFileName aOutputName(pInput->m_acOutMrcFile);
	aInputName.GetName(m_acInMrcFile);
	aOutputName.GetFolder(m_acOutFolder);
	//-----------------------------------
	mSaveImages();
	mSaveFittings();
}

void CSaveCtfResults::mSaveImages(void)
{
	CCtfResults* pCtfResults = CCtfResults::GetInstance();
	Mrc::CSaveMrc aSaveMrc;
	bool bClean = true;
	//----------------------------------------------------
	char acCtfImgFile[256] = {'\0'};
	strcpy(acCtfImgFile, m_acOutFolder);
	strcat(acCtfImgFile, m_acInMrcFile);
	strcat(acCtfImgFile, "_Ctf.mrc");
	//----------------------------------------
	aSaveMrc.OpenFile(acCtfImgFile);
	aSaveMrc.SetMode(Mrc::eMrcFloat);
	aSaveMrc.SetImgSize(pCtfResults->m_aiSpectSize,
	   pCtfResults->m_iNumImgs, 1, 1.0f);
	//-----------------------------------
	for(int i=0; i<pCtfResults->m_iNumImgs; i++)
	{	float* pfSpect = pCtfResults->GetSpect(i, bClean);
		aSaveMrc.DoIt(i, pfSpect);
		if(pfSpect != 0L) delete[] pfSpect;
	}
	aSaveMrc.CloseFile();
}

void CSaveCtfResults::mSaveFittings(void)
{
	CCtfResults* pCtfResults = CCtfResults::GetInstance();
	char acCtfTxtFile[256] = {'\0'};
	strcpy(acCtfTxtFile, m_acOutFolder);
        strcat(acCtfTxtFile, m_acInMrcFile);
        strcat(acCtfTxtFile, "_Ctf.txt");
	//-------------------------------
	FILE* pFile = fopen(acCtfTxtFile, "w");
	if(pFile == 0L) return;
	//---------------------
	fprintf(pFile, "# Columns: #1 micrograph number; "
	   "#2 - defocus 1 [A]; #3 - defocus 2; "
	   "#4 - azimuth of astigmatism; "
	   "#5 - additional phase shift [radian]; "
	   "#6 - cross correlation; "
	   "#7 - spacing (in Angstroms) up to which CTF rings were "
	   "fit successfully\n");
	for(int i=0; i<pCtfResults->m_iNumImgs; i++)
	{	fprintf(pFile, "%4d   %6.2f  %8.2f  %8.2f  %8.2f  "
		   "%7.2f  %8.4f\n", i+1,
		   pCtfResults->GetDfMax(i),
		   pCtfResults->GetDfMin(i),
		   pCtfResults->GetAzimuth(i),
		   pCtfResults->GetExtPhase(i) * s_fD2R,
		   pCtfResults->GetScore(i),
		   pCtfResults->GetCtfRes(i));
        }
        fclose(pFile);
}
