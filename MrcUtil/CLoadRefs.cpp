#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <Mrcfile/CMrcFileInc.h>
#include <memory.h>
#include <string.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

CLoadRefs* CLoadRefs::m_pInstance = 0L;

CLoadRefs* CLoadRefs::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CLoadRefs;
	return m_pInstance;
}

void CLoadRefs::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CLoadRefs::CLoadRefs(void)
{
	m_pfGain = 0L;
	m_pfDark = 0L;
}

CLoadRefs::~CLoadRefs(void)
{
	this->CleanRefs();
}

void CLoadRefs::CleanRefs(void)
{
	if(m_pfGain != 0L) cudaFreeHost(m_pfGain);
	if(m_pfDark != 0L) cudaFreeHost(m_pfDark);
	m_pfGain = 0L;
	m_pfDark = 0L;
}

bool CLoadRefs::LoadGain(char* pcMrcFile)
{	
	mClearGain();
	memset(m_aiRefSize, 0, sizeof(m_aiRefSize));
	//------------------------------------------
	Mrc::CLoadMrc aLoadMrc;
	bool bOpen = aLoadMrc.OpenFile(pcMrcFile);
	if(!bOpen) return false;
	//----------------------
	int iMode = aLoadMrc.m_pLoadMain->GetMode();
	if(iMode != Mrc::eMrcFloat) return false;
	//---------------------------------------
	int iSizeX = aLoadMrc.m_pLoadMain->GetSizeX();
	int iSizeY = aLoadMrc.m_pLoadMain->GetSizeY();
	int iPixels = iSizeX * iSizeY;
	if(iPixels <= 0) return false;
	//----------------------------
	printf("Load gain refereince from\n"
	   "   %s\n", pcMrcFile);
	cudaMallocHost(&m_pfGain, sizeof(float) * iPixels);
	aLoadMrc.m_pLoadImg->DoIt(0, m_pfGain);
	m_aiRefSize[0] = iSizeX;
	m_aiRefSize[1] = iSizeY;
	//----------------------
	mCheckDarkRef();
	printf("Gain reference has been loaded.\n\n");
	return true;
}

bool CLoadRefs::LoadDark(char* pcMrcFile)
{	
	mClearDark();
	memset(m_aiDarkSize, 0, sizeof(m_aiDarkSize));
	//--------------------------------------------
	Mrc::CLoadMrc aLoadMrc;
	bool bOpen = aLoadMrc.OpenFile(pcMrcFile);
	if(!bOpen) return false;
	//----------------------
	printf("Load dark reference from\n"
	   "   %s\n", pcMrcFile);
	int aiDarkSize[2];
	m_aiDarkSize[0] = aLoadMrc.m_pLoadMain->GetSizeX();
	m_aiDarkSize[1] = aLoadMrc.m_pLoadMain->GetSizeY();
	int iMode = aLoadMrc.m_pLoadMain->GetMode();
	//------------------------------------------
	if(iMode == Mrc::eMrcFloat)
	{	int iPixels = m_aiDarkSize[0] * m_aiDarkSize[1];
		int iBytes = sizeof(float) * iPixels;
		cudaMallocHost(&m_pfDark, iBytes);
		memset(m_pfDark, 0, iBytes);
		aLoadMrc.m_pLoadImg->DoIt(0, m_pfDark);
	} 
	else
	{	void* pvDark = aLoadMrc.m_pLoadImg->DoIt(0);
		if(pvDark == 0L) return false;
		m_pfDark = mToFloat(pvDark, iMode, m_aiDarkSize);
		delete[] (char*)pvDark;
	}
	//-----------------------------
	printf("Dark reference has been loaded.\n\n");
	mCheckDarkRef();	
	return true;
}

void CLoadRefs::PostProcess(int iRotFact, int iFlip, int iInverse)
{
	if(m_pfGain == 0L) return;
	if(iRotFact == 0 && iFlip == 0 && iInverse == 0) return;
	//------------------------------------------------------
	float* gfBuf = 0L;
	size_t tBytes = sizeof(float) * m_aiRefSize[0] * m_aiRefSize[1];
	cudaMalloc(&gfBuf, tBytes);
	//-------------------------
	if(m_pfGain != 0L)
	{	cudaMemcpy(gfBuf, m_pfGain, tBytes, cudaMemcpyDefault);
		mRotate(gfBuf, m_aiRefSize, iRotFact);
		mFlip(gfBuf, m_aiRefSize, iFlip);
		mInverse(gfBuf, m_aiRefSize, iInverse);
		cudaMemcpy(m_pfGain, gfBuf, tBytes, cudaMemcpyDefault);
	}
	//-------------------------------------------------------------
	if(m_pfDark != 0L && (iRotFact != 0 || iFlip == 0))
	{	cudaMemcpy(gfBuf, m_pfDark, tBytes, cudaMemcpyDefault);
		mRotate(gfBuf, m_aiDarkSize, iRotFact);
		mFlip(gfBuf, m_aiDarkSize, iFlip);
		cudaMemcpy(m_pfDark, gfBuf, tBytes, cudaMemcpyDefault);
	}
	//-------------------------------------------------------------
	cudaFree(gfBuf);
}

bool CLoadRefs::AugmentRefs(int* piFmSize)
{
	if(m_pfGain == 0L && m_pfDark == 0L) return true;
	if(piFmSize[0] == m_aiRefSize[0] && 
	   piFmSize[1] == m_aiRefSize[1]) return true;
	//--------------------------------------------
	int iFactX = piFmSize[0] / m_aiRefSize[0];
	int iModX = piFmSize[0] % m_aiRefSize[0];
	int iFactY = piFmSize[1] / m_aiRefSize[1];
	int iModY = piFmSize[1] % m_aiRefSize[1];
	if(iFactX != iFactY || iModX != 0 || iModY != 0)
	{	fprintf(stderr, "Error: gain size does not match frame size.\n"
		   "        Unable to apply gain reference.\n\n");
		return false;
	}
	//-------------------
	float* pfAugGain = mAugmentRef(m_pfGain, iFactX);
	if(m_pfGain != 0L) cudaFreeHost(m_pfGain);
	m_pfGain = pfAugGain;
	//-------------------
	float* pfAugDark = mAugmentRef(m_pfDark, iFactX);
	if(m_pfDark != 0L) cudaFreeHost(m_pfDark);
	m_pfDark = pfAugDark;
	//-------------------	
	m_aiRefSize[0] *= iFactX;
	m_aiRefSize[1] *= iFactX;
	m_aiDarkSize[0] *= iFactX;
	m_aiDarkSize[1] *= iFactY;
	return true;
}

void CLoadRefs::mClearGain(void)
{
	if(m_pfGain == 0L) return;
	cudaFreeHost(m_pfGain);
	m_pfGain = 0L;
}

void CLoadRefs::mClearDark(void)
{
	if(m_pfDark == 0L) return;
	cudaFreeHost(m_pfDark);
	m_pfDark = 0L;
}

void CLoadRefs::mRotate(float* gfRef, int* piRefSize, int iRotFact)
{
	if(iRotFact == 0) return;
	//-----------------------
	bool bGpu = true;
	G90Rotate2D aRot2D;
	aRot2D.Setup(piRefSize, iRotFact);
	aRot2D.DoIt(gfRef, bGpu);
	aRot2D.GetRotImg(gfRef, bGpu);
	//----------------------------
	piRefSize[0] = aRot2D.m_aiRotSize[0];
	piRefSize[1] = aRot2D.m_aiRotSize[1];
}

void CLoadRefs::mFlip(float* gfRef, int* piRefSize, int iFlip)
{    
	if(iFlip == 0) return;
	//--------------------
	bool bGpu = true;
	GFlip2D aGFlip2D;
	if(iFlip == 1) aGFlip2D.Vertical(gfRef, bGpu, piRefSize);
	else if(iFlip == 2) aGFlip2D.Horizontal(gfRef, bGpu, piRefSize);
}

void CLoadRefs::mInverse(float* gfRef, int* piRefSize, int iInverse)
{
	if(iInverse == 0) return;
	//-----------------------
	GInverse2D aGInverse2D;
	aGInverse2D.DoIt(gfRef, true, piRefSize);
}

float* CLoadRefs::mToFloat(void* pvRef, int iMode, int* piSize)
{
	float* pfRef = 0L;
	int iPixels = piSize[0] * piSize[1];
	//----------------------------------
	if(iMode == 0)
	{	char* pcRef = (char*)pvRef;
		cudaMallocHost(&pfRef, sizeof(float) * iPixels);
		for(int i=0; i<iPixels; i++) pfRef[i] = pcRef[i];
		return pfRef;
	}
	//----------------
	if(iMode == Mrc::eMrcShort)
	{	short* psRef = (short*)pvRef;
		cudaMallocHost(&pfRef, sizeof(float) * iPixels);
		for(int i=0; i<iPixels; i++) pfRef[i] = psRef[i];
		return pfRef;
	}
	//----------------
	if(iMode == Mrc::eMrcUShort)
	{	unsigned short* pusRef = (unsigned short*)pvRef;
		cudaMallocHost(&pfRef, sizeof(float) * iPixels);
		for(int i=0; i<iPixels; i++) pfRef[i] = pusRef[i];
		return pfRef;
	}
	//----------------
	if(iMode == Mrc::eMrcUCharEM)
	{	unsigned char* pucRef = (unsigned char*)pvRef;
		cudaMallocHost(&pfRef, sizeof(float) * iPixels);
		for(int i=0; i<iPixels; i++) pfRef[i] = pucRef[i];
		return pfRef;
	}
	return 0L;
}

void CLoadRefs::mCheckDarkRef(void)
{
	if(m_pfDark == 0L) return;
	if(m_aiDarkSize[0] == m_aiRefSize[0] && 
	   m_aiDarkSize[1] == m_aiRefSize[1]) return;
	//-------------------------------------------
	cudaFreeHost(m_pfDark);
	m_pfDark = 0L;
	memset(m_aiDarkSize, 0, sizeof(m_aiDarkSize));
	printf("Warning: The sizes of Dark and gain references do not "
	   "match.\n   Dark reference has been removed.\n\n");
}

float* CLoadRefs::mAugmentRef(float* pfRef, int iFact)
{
	if(pfRef == 0L) return 0L;
	int aiAugSize[] = {m_aiRefSize[0] * iFact, m_aiRefSize[1] * iFact};
	size_t tBytes = sizeof(float) * aiAugSize[0] * aiAugSize[1];
	float* pfAugRef = 0L;
	cudaMallocHost(&pfAugRef, tBytes);
	//--------------------------------
	GAugmentRef aGAugmentRef;
	aGAugmentRef.DoIt(pfRef, m_aiRefSize, pfAugRef, aiAugSize);
	return pfAugRef;
}

