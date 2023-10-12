#include "CAlignInc.h"
#include <memory.h>
#include <math.h>
#include <stdio.h>
#include <nvToolsExt.h>

using namespace MotionCor2::Align;

CPatchShifts::CPatchShifts(void)
{
	m_iNumPatches = 0;
	m_pFullShift = 0L;
	m_pfPatCenters = 0L;
	m_pfPatShifts = 0L;
	m_pbBadShifts = 0L;
}

CPatchShifts::~CPatchShifts(void)
{
	mClean();
}

void CPatchShifts::Setup(int iNumPatches, int* piFullSize)
{
	mClean();
	//-------
	m_iNumPatches = iNumPatches;
	m_aiFullSize[0] = piFullSize[0];
	m_aiFullSize[1] = piFullSize[1];
	m_aiFullSize[2] = piFullSize[2];
	if(m_iNumPatches <= 1) return;
	//----------------------------
	int iBytes = iNumPatches * 2 * sizeof(float);
	cudaMallocHost(&m_pfPatCenters, iBytes);
	//--------------------------------------
	int iNumPoints = iNumPatches * m_aiFullSize[2];
	iBytes = iNumPoints * (2 * sizeof(float) + sizeof(bool));
	cudaMallocHost(&m_pfPatShifts, iBytes);
	m_pbBadShifts = (bool*)(m_pfPatShifts + iNumPoints * 2);
}

void CPatchShifts::Setup(int iPatchesX, int iPatchesY, int* piFullSize)
{
	int iNumPatches = iPatchesX * iPatchesY;
	this->Setup(iNumPatches, piFullSize);
}

void CPatchShifts::SetRawShift(CStackShift* pStackShift, int iPatch)
{
	if(iPatch < 0 || iPatch >= m_iNumPatches) return;
	//-----------------------------------------------
	int iOffset = m_iNumPatches * 2;
	float* pfDstShift = m_pfPatShifts + iPatch * 2;
	pStackShift->GetShift(0, pfDstShift);
	for(int i=1; i<m_aiFullSize[2]; i++)
	{	pfDstShift = pfDstShift + iOffset;
		pStackShift->GetShift(i, pfDstShift);
	}
	//-------------------------------------------
	float* pfDstCenter = m_pfPatCenters + iPatch * 2;
	pStackShift->GetCenter(pfDstCenter);
}

void CPatchShifts::SetFullShift(CStackShift* pFullShift)
{
	if(m_pFullShift == pFullShift) return;
	if(m_pFullShift != 0L) delete m_pFullShift;
	m_pFullShift = pFullShift;
}

void CPatchShifts::GetLocalShift
(	int iFrame, 
	int iPatch,
	float* pfShift
)
{	int iOffset = (iFrame * m_iNumPatches + iPatch) * 2;
	pfShift[0] = m_pfPatShifts[iOffset];
	pfShift[1] = m_pfPatShifts[iOffset + 1];
}

void CPatchShifts::GetPatchCenter
(	int iPatch,
	float* pfCenter
)
{	pfCenter[0] = m_pfPatCenters[2 * iPatch];
	pfCenter[1] = m_pfPatCenters[2 * iPatch + 1];	
}

void CPatchShifts::CopyShiftsToGpu(float* gfPatShifts)
{
	int iBytes = m_iNumPatches * m_aiFullSize[2] * sizeof(float) * 2;
	cudaMemcpy(gfPatShifts, m_pfPatShifts, iBytes, cudaMemcpyDefault);
}

void CPatchShifts::CopyFlagsToGpu(bool* gbBadShifts)
{
	int iBytes = m_iNumPatches * m_aiFullSize[2] * sizeof(bool);
	cudaMemcpy(gbBadShifts, m_pbBadShifts, iBytes, cudaMemcpyDefault);
}

void CPatchShifts::CopyCentersToGpu(float* gfPatCenters)
{
	int iBytes = m_iNumPatches * 2 * sizeof(float);
	cudaMemcpy(gfPatCenters, m_pfPatCenters, iBytes, cudaMemcpyDefault);
}

void CPatchShifts::CalcShiftSigma(int iFrame, float* pfSigmaXY)
{
	pfSigmaXY[0] = 0.0f;
	pfSigmaXY[1] = 0.0f;
	//-----------------
	int iCount = 0;
	float afMean[2] = {0.0f}, afVar[2] = {0.0f};
	float* pfPatShifts = m_pfPatShifts + iFrame * m_iNumPatches * 2;
	//--------------------------------------------------------------
	for(int i=0; i<m_iNumPatches; i++)
	{	if(m_pbBadShifts[i]) continue;
		float fSx = pfPatShifts[i * 2];
		float fSy = pfPatShifts[i * 2 + 1];
		afMean[0] += fSx;
		afMean[1] += fSy;
		afVar[0] += (fSx * fSx);
		afVar[1] += (fSy * fSy);
		iCount += 1;
	}
	if(iCount == 0) return;
	//---------------------
	float fFact = 1.0f / (iCount + (float)1e-20);
	afMean[0] *= fFact;
	afMean[1] *= fFact;
	afVar[0] = afVar[0] * fFact - afMean[0] * afMean[0];
	afVar[1] = afVar[1] * fFact - afMean[1] * afMean[1];
	if(afVar[0] > 0) pfSigmaXY[0] = (float)sqrtf(afVar[0]);
	if(afVar[1] > 1) pfSigmaXY[1] = (float)sqrtf(afVar[1]);
}
	
void CPatchShifts::LogFullShifts(char* pcLogFile)
{
	if(m_pFullShift == 0L) return;
	if(pcLogFile == 0L) return;
	if(strlen(pcLogFile) == 0L) return;
	//---------------------------------
	char acLogFile[256];
	sprintf(acLogFile, "%s-Patch-Full.log", pcLogFile);
	FILE* pFile = fopen(acLogFile, "wt");
	if(pFile == 0L) return;
	//---------------------
	fprintf(pFile, "# Patch based alignment\n");
	fprintf(pFile, "# Initial alignment based upon full frames\n\n");
	//---------------------------------------------------------------
	float afShift[2] = {0.0f};
	const char* pcFormat = "%4d  %8.2f  %8.2f\n";
	for(int i=0; i<m_pFullShift->m_iNumFrames; i++)
	{	m_pFullShift->GetShift(i, afShift);
		fprintf(pFile, pcFormat, i+1, afShift[0], afShift[1]);
	}
	fclose(pFile);
}	

void CPatchShifts::LogPatchShifts(char* pcLogFile)
{
	if(m_iNumPatches == 0) return;
	if(pcLogFile == 0L || strlen(pcLogFile) == 0) return;
	//---------------------------------------------------
	char acLogFile[256];
	sprintf(acLogFile, "%s-Patch-Patch.log", pcLogFile);
	FILE* pFile = fopen(acLogFile, "wt");
	if(pFile == 0L) return;	
	//---------------------
	fprintf(pFile, "# Patch based alignment\n");
	fprintf(pFile, "# Number of patches: %d\n", m_iNumPatches);
	fprintf(pFile, "# Shifts are listed per patch.\n");
	fprintf(pFile, "# Movie size: %d  %d  %d\n\n", m_aiFullSize[0],
	   m_aiFullSize[1], m_aiFullSize[2]);
	//-----------------------------------
	float afRawShift[2] = {0.0f}, afFitShift[2] = {0.0f};
	const char* pcFormat1 = "%4d %8.2f %8.2f ";
	const char* pcFormat2 = "%8.2f %8.2f %4d\n";
	//------------------------------------------
	for(int i=0; i<m_iNumPatches; i++)
	{	fprintf(pFile, "#Patch %03d raw and fit shifts\n", i+1);
		float* pfPatCent = m_pfPatCenters + i * 2;
		//----------------------------------------
		for(int j=0; j<m_aiFullSize[2]; j++)
		{	float* pfPatShift = m_pfPatShifts
			   + (j * m_iNumPatches + i) * 2;
			fprintf(pFile, pcFormat1, j+1, pfPatCent[0], 
			   pfPatCent[1]);
			fprintf(pFile, pcFormat2, pfPatShift[0], pfPatShift[1], 
			   m_pbBadShifts[j * m_iNumPatches + i]);
		}
		fprintf(pFile, "\n");
	}
	fclose(pFile);
}

void CPatchShifts::LogFrameShifts(char* pcLogFile)
{
	if(m_iNumPatches == 0) return;
	if(pcLogFile == 0L) return;
	if(strlen(pcLogFile) == 0) return;
	//--------------------------------
	char acLogFile[256];
	sprintf(acLogFile, "%s-Patch-Frame.log", pcLogFile);
	FILE* pFile = fopen(acLogFile, "wt");
	if(pFile == 0L) return;
	//---------------------
	fprintf(pFile, "# Patch based alignment\n");
	fprintf(pFile, "# Number of patches: %d\n", m_iNumPatches);
	fprintf(pFile, "# Shifts are listed per frame.\n");
	fprintf(pFile, "# Movie size: %d  %d  %d\n\n", m_aiFullSize[0],
	   m_aiFullSize[1], m_aiFullSize[2]);
	//-----------------------------------
	float fCentX, fCentY;
	float afRawShift[2] = {0.0f}, afFitShift[2] = {0.0f};
	const char* pcFormat1 = "%4d %8.2f %8.2f ";
        const char* pcFormat2 = "%8.2f %8.2f %4d\n";
        //------------------------------------------
	for(int i=0; i<m_aiFullSize[2]; i++)
	{	for(int j=0; j<m_iNumPatches; j++)
		{	float* pfCent = m_pfPatCenters + j * 2;
			float* pfShift = m_pfPatShifts +
			   (i * m_iNumPatches + j) * 2;
			fprintf(pFile, pcFormat1, i+1, pfCent[0], pfCent[1]);
			fprintf(pFile, pcFormat2, pfShift[0], pfShift[1],
			   m_pbBadShifts[i * m_iNumPatches + j]);
		}
		fprintf(pFile, "\n");
	}
	fclose(pFile);
}

void CPatchShifts::mClean(void)
{
	if(m_pFullShift != 0L) delete m_pFullShift;
	m_pFullShift = 0L;
	//----------------
	if(m_pfPatCenters != 0L) cudaFreeHost(m_pfPatCenters);
	if(m_pfPatShifts != 0L) cudaFreeHost(m_pfPatShifts);
	m_pfPatCenters = 0L;
	m_pfPatShifts = 0L;
	m_pbBadShifts = 0L;	
}

void CPatchShifts::MakeRelative(void)
{
	float afMeanStd[2], fMin = (float)1e20;
	int iRef = 0;
	for(int i=1; i<m_aiFullSize[2]; i++)
	{	mCalcMeanStd(i, afMeanStd);
		float fVal = afMeanStd[0] + afMeanStd[1];
		printf("%3d  %3d %8.2f  %8.2f\n", i, iRef, afMeanStd[0],
		   afMeanStd[1]); 
		if(fVal > fMin) continue;
		fMin = fVal;
		iRef = i;
	}
	//---------------
	float* pfRefShifts = m_pfPatShifts + iRef * m_iNumPatches * 2;
	for(int i=0; i<m_aiFullSize[2]; i++)
	{	float* pfPatShifts = m_pfPatShifts + i * m_iNumPatches * 2;
		for(int j=0; j<m_iNumPatches; j++)
		{	int k = j * 2;
			pfPatShifts[k] -= pfRefShifts[k];
			pfPatShifts[k+1] -= pfRefShifts[k+2];
		}
	}
}

void CPatchShifts::DetectBads(void)
{
	for(int i=0; i<m_aiFullSize[2]; i++)
	{	mDetectBadOnFrame(i);
	}
}

void CPatchShifts::mDetectBadOnFrame(int iFrame)
{
	float* pfShifts = m_pfPatShifts + iFrame * m_iNumPatches * 2;
	double dFmRms = 0.0;
	int iCount = 0;
	for(int i=0; i<m_iNumPatches; i++)
	{	float* pfRefShift = pfShifts + i * 2;
		for(int j=i+1; j<m_iNumPatches; j++)
		{	float fX = pfShifts[j*2] - pfRefShift[0];
			float fY = pfShifts[j*2+1] - pfRefShift[1];
			dFmRms += (fX * fX + fY * fY);
			iCount += 1;
		}
	}
	dFmRms = sqrtf(dFmRms / iCount);
	float fTol = (float)(dFmRms * 2);
	//printf("**** Frame RMS: %4d  %8.2f\n", iFrame, dFmRms);
	//-----------------------------------------------------
	bool* pbBads = m_pbBadShifts + iFrame * m_iNumPatches;
	for(int i=0; i<m_iNumPatches; i++)
	{	float fLocalRms = mCalcLocalRms(iFrame, i);
		if(fLocalRms < fTol) pbBads[i] = false;
		else pbBads[i] = true;
	}
}

float CPatchShifts::mCalcLocalRms(int iFrame, int iPatch)
{
	float* pfShifts = m_pfPatShifts + iFrame * m_iNumPatches * 2;
	float* pfCentShift = pfShifts + iPatch * 2;
	float* pfCent = m_pfPatCenters + iPatch * 2;
	//------------------------------------------
	double dRms = 0, dSumW = 0;
	for(int i=0; i<m_iNumPatches; i++)
	{	if(i == iPatch) continue;
		int j0 = i * 2;
		int j1 = j0 + 1;
		float fX = (m_pfPatCenters[j0] - pfCent[0]) / m_aiFullSize[0];
		float fY = (m_pfPatCenters[j1] - pfCent[1]) / m_aiFullSize[1];
		double dW = expf(-100.0 * (fX * fX + fY * fY));
		//-------------------------------------------
		fX = pfShifts[j0] - pfCentShift[0];
		fY = pfShifts[j1] - pfCentShift[1];
		dRms += (dW * (fX * fX + fY * fY));
		dSumW += dW;
	}
	dRms = sqrtf(dRms / dSumW);
	return (float)dRms;
}

void CPatchShifts::mCalcMeanStd(int iFrame, float* pfMeanStd)
{
	float* pfShifts = m_pfPatShifts + iFrame * m_iNumPatches * 2;
	//-----------------------------------------------------------
	float fMeanX = 0.0f, fMeanY = 0.0f;
	for(int i=0; i<m_iNumPatches; i++)
	{	int j = i * 2;
		fMeanX += pfShifts[j];
		fMeanY += pfShifts[j + 1];
	}
	fMeanX /= m_iNumPatches;
	fMeanY /= m_iNumPatches;
	//----------------------
	float fStd = 0.0f;
	for(int i=0; i<m_iNumPatches; i++)
	{	int j = 2 * i;
		float fX = pfShifts[j] - fMeanX;
		float fY = pfShifts[j+1] - fMeanY;
		fStd += (fX * fX + fY * fY);
	}
	fStd = (float)sqrt(fStd / m_iNumPatches);
	float fMean = (float)sqrt(fMeanX * fMeanX + fMeanY * fMeanY);
	//-----------------------------------------------------------
	pfMeanStd[0] = fMean;
	pfMeanStd[1] = fStd;
} 		
