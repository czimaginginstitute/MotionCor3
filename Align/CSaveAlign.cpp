#include "CAlignInc.h"
#include <memory.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace MotionCor2::Align;

CSaveAlign* CSaveAlign::m_pInstance = 0L;

CSaveAlign* CSaveAlign::GetInstance(void)
{
     if(m_pInstance != 0L) return m_pInstance;
     m_pInstance = new CSaveAlign;
     return m_pInstance;
}

void CSaveAlign::DeleteInstance(void)
{
     if(m_pInstance == 0L) return;
     delete m_pInstance;
     m_pInstance = 0L;
}

CSaveAlign::CSaveAlign(void)
{
     strcpy(m_acSetting, "setting");
     strcpy(m_acStackSize, "stackSize");
     strcpy(m_acPatches, "patches");
     strcpy(m_acThrow, "throw");
     strcpy(m_acGlobalShift, "globalShift");
     strcpy(m_acLocalShift, "localShift");
     strcpy(m_acConverge, "Converge");
     strcpy(m_acStackID, "stackID");
     strcpy(m_acPatchID, "patchID");
     //-----------------------------
     m_pFile = 0L;
}

CSaveAlign::~CSaveAlign(void)
{
     if(m_pFile != 0L) fclose(m_pFile);
}

bool CSaveAlign::Open(char* pcFileName)
{
     if(m_pFile != 0L) fclose(m_pFile);
     m_pFile = 0L;
     //-----------
     if(pcFileName == 0L || strlen(pcFileName) == 0) return false;
     //-----------------------------------------------------------
     m_pFile = fopen(pcFileName, "wt");
     if(m_pFile != 0L) return true;
     //----------------------------
     printf("Info: %s\n%s\n\n",
       "      cannot be opened. Alignment wiil not be saved.", 
       pcFileName);
     return false;
}

bool CSaveAlign::Open(char* pcDirName, char* pcStackFile)
{
     if(m_pFile != 0L) fclose(m_pFile);
     m_pFile = 0L;
     //-----------
     int iSize = strlen(pcDirName);
     if(pcDirName == 0L || iSize == 0) return false;
     //---------------------------------------------------------
     char acFileName[256] = {'\0'};
     strcpy(acFileName, pcDirName);
     if(pcDirName[iSize - 1] != '/') strcat(acFileName, "/");
     //------------------------------------------------------
     char* pcSlash = strrchr(pcStackFile, '/');
     char* pcFileName = (pcSlash == 0L) ? pcStackFile : pcSlash+1;
     strcat(acFileName, pcFileName);
     //-----------------------------
     char* pcDot = strrchr(acFileName, '.');
     if(pcDot == 0L) strcat(acFileName, ".aln");
     else strcpy(pcDot, ".aln");
     //-------------------------
     return this->Open(acFileName);
}

void CSaveAlign::SaveSetting
(	int* piStkSize,
	int* piPatches,
	int* piThrow
)
{	if(m_pFile == 0L) return;
	//-----------------------
	fseek(m_pFile, 0L, SEEK_SET);
	fprintf(m_pFile, "%s\n", m_acSetting);
	fprintf(m_pFile, "   %s: %d %d %d %d\n", m_acStackSize,
	   piStkSize[0], piStkSize[1], piStkSize[2], 1);
	fprintf(m_pFile, "   %s: %d %d %d\n", m_acPatches,
	   piPatches[0], piPatches[1], piPatches[2]);
	fprintf(m_pFile, "   %s: %d %d\n", m_acThrow, 
	   piThrow[0], piThrow[1]);      
}

void CSaveAlign::DoGlobal(CStackShift* pFullShift)
{
	if(m_pFile == 0L) return;
	mSaveGlobal(pFullShift);
}

void CSaveAlign::DoLocal(CPatchShifts* pPatchShifts)
{
	if(m_pFile == 0L) return;
	mSaveGlobal(pPatchShifts->m_pFullShift);
	mSaveLocal(pPatchShifts);
}

void CSaveAlign::mSaveGlobal(CStackShift* pFullShift)
{
	fprintf(m_pFile, "#    Global Shift\n");
	fprintf(m_pFile, "#---------------------\n");
	fprintf(m_pFile, "%s\n", m_acGlobalShift);
	fprintf(m_pFile, "   %s %d\n", m_acStackID, 0);
	//---------------------------------------------
	float afShift[2] = {0.0f};
	for(int i=0; i<pFullShift->m_iNumFrames; i++)
	{	pFullShift->GetShift(i, afShift);
		fprintf(m_pFile, "%6d  %+8.3f  %+8.3f\n", 
		   i, afShift[0], afShift[1]);
     	}
}

void CSaveAlign::mSaveLocal(CPatchShifts* pPatchShifts)
{
	fprintf(m_pFile, "#    Local Shift\n");
	fprintf(m_pFile, "#---------------------\n");
	fprintf(m_pFile, "%s\n", m_acLocalShift);
	//---------------------------------------
	float afPatCent[2] = {0.0f}, afShift[2] = {0.0f};
	int iLastPatch = pPatchShifts->m_iNumPatches - 1;
	//-----------------------------------------------
	for(int i=0; i<pPatchShifts->m_iNumPatches; i++)
	{	fprintf(m_pFile, "   %s: %d\n", m_acPatchID, i);
		fprintf(m_pFile, "   %s: %d\n", m_acConverge, 1);
		//-----------------------------------------------
		pPatchShifts->GetPatchCenter(i, afPatCent);
		for(int j=0; j<pPatchShifts->m_aiFullSize[2]; j++)
		{	pPatchShifts->GetLocalShift(j, i, afShift);
			fprintf(m_pFile, "%6d  %8.2f  %8.2f  %+8.3f  %+8.3f\n",
			   j, afPatCent[0], afPatCent[1], 
			   afShift[0], afShift[1]);
		}
		if(i < iLastPatch) fprintf(m_pFile, "\n");
	}
}


