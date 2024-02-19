#include "CMainInc.h"
#include "Util/CUtilInc.h"
#include "MrcUtil/CMrcUtilInc.h"
#include "TiffUtil/CTiffFileInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <Mrcfile/CMrcFileInc.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory.h>
#include <string.h>
#include <stdio.h>

using namespace MotionCor2;
namespace MU = MotionCor2::MrcUtil;
namespace TU = MotionCor2::TiffUtil;

CLoadRefs* CLoadRefs::m_pInstance = 0L;

static bool sCheckMode(int iMode, int iRefType)
{
	if(iMode == Mrc::eMrcFloat) return true;
	//-----------------
	const char* pcRefType = (iRefType == 1) ? "Gain" : "Dark";
	printf("Warning: invalid pixel type of %s reference: %d\n"
	   "   mode 2 (32-bit float) is expected.\n\n",
	   pcRefType, iMode);
	printf("%s reference will not be loaded.\n"
	   "   %s correction will be skipped.\n\n", 
	   pcRefType, pcRefType);
	return false;
}

static bool sCheckSize(int iSizeX, int iSizeY, int iRefType)
{
	if(iSizeX >= 32 && iSizeX <= 32000 &&
	   iSizeY >= 32 && iSizeY <= 32000) return true;
	//-----------------
	const char* pcRefType = (iRefType == 1) ? "Gain" : "Dark";
	printf("Warning: invalid size of %s reference: %d  %d\n\n",
	   pcRefType, iSizeX, iSizeY);
	printf("%s reference will not be loaded.\n"
	   "   %s correction will be skipped.\n\n", 
	   pcRefType, pcRefType);
	return false;
}

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
	m_bAugmented = false;
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

bool CLoadRefs::LoadGain(char* pcGainFile)
{	
	mClearGain();
	memset(m_aiRefSize, 0, sizeof(m_aiRefSize));
	//-----------------
	int iFileType = mGetFileType(pcGainFile);
	if(iFileType == 1) mLoadGainMrc(pcGainFile);
	else if(iFileType == 2) mLoadGainTiff(pcGainFile);
	//-----------------
	if(m_pfGain == 0L) return false;
	else return true;
}

int CLoadRefs::mGetFileType(char* pcRefFile)
{
	if(pcRefFile == 0L) return -1;
	char* pcExt = strrchr(pcRefFile, '.');
	if(pcExt == 0L) return -2;
	//-----------------
	char* pcMrc = strcasestr(pcExt, "mrc");
	if(pcMrc != 0L) return 1;
	//-----------------
	char* pcTiff = strcasestr(pcExt, "tif");
	if(pcTiff != 0L) return 2;
	//-----------------
	char* pcGain = strcasestr(pcExt, "gain");
	if(pcGain != 0L) return 2;
	//-----------------
	return -3;
}

void CLoadRefs::mLoadGainMrc(char* pcMrcFile)
{
	Mrc::CLoadMrc aLoadMrc;
	bool bOpen = aLoadMrc.OpenFile(pcMrcFile);
	if(!bOpen) return;
	//-----------------
	int iMode = aLoadMrc.m_pLoadMain->GetMode();
	if(!sCheckMode(iMode, 1)) return;
	//-----------------
	int iSizeX = aLoadMrc.m_pLoadMain->GetSizeX();
	int iSizeY = aLoadMrc.m_pLoadMain->GetSizeY();
	if(!sCheckSize(iSizeX, iSizeY, 1)) return;
	//-----------------
	printf("Load gain refereince from\n"
	   "   %s\n", pcMrcFile);
	//-----------------
	int iPixels = iSizeX * iSizeY;
	cudaMallocHost(&m_pfGain, sizeof(float) * iPixels);
	aLoadMrc.m_pLoadImg->DoIt(0, m_pfGain);
	m_aiRefSize[0] = iSizeX;
	m_aiRefSize[1] = iSizeY;
	//----------------------
	mCheckDarkRef();
	printf("Gain reference has been loaded.\n\n");
}

void CLoadRefs::mLoadGainTiff(char* pcTiffFile)
{
	int iFile = open(pcTiffFile, O_RDONLY);
	if(iFile == -1) return;
	//-----------------
	TiffUtil::CLoadTiffHeader loadHeader;
	loadHeader.DoIt(iFile);
	int iMode = loadHeader.GetMode();
	if(!sCheckMode(iMode, 1)) return;
	//-----------------
	int aiSize[3] = {0};
	loadHeader.GetSize(aiSize, 2);
	if(!sCheckSize(aiSize[0], aiSize[1], 1)) return;
	//-----------------
	printf("Load gain reference from\n   %s\n", pcTiffFile);
	int iPixels = aiSize[0] * aiSize[1];
	cudaMallocHost(&m_pfGain, sizeof(float) * iPixels);
	//-----------------
	TiffUtil::CLoadTiffImage loadImage;
	loadImage.SetFile(iFile);
	loadImage.DoIt(0, (char*)m_pfGain);
	//-----------------
	m_aiRefSize[0] = aiSize[0];
	m_aiRefSize[1] = aiSize[1];
	//-----------------
	mCheckDarkRef();
	close(iFile);
	printf("Gain reference has been loaded.\n\n");
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
	bool bNoRef = (m_pfGain == 0L) && (m_pfDark == 0L);
	bool bSame = (piFmSize[0] == m_aiRefSize[0]) && 
	   (piFmSize[1] == m_aiRefSize[1]);
	if(m_bAugmented || bNoRef || bSame) return true;
	//-----------------
	int iFactX = piFmSize[0] / m_aiRefSize[0];
	int iModX = piFmSize[0] % m_aiRefSize[0];
	int iFactY = piFmSize[1] / m_aiRefSize[1];
	int iModY = piFmSize[1] % m_aiRefSize[1];
	//-----------------
	if(iFactX == 0 || iFactY == 0 || iFactX != iFactY || 
	   iModX != 0 || iModY != 0)
	{	fprintf(stderr, "Error: cannot augment gain reference.\n"
		   "   Frame size: %d  %d\n"
		   "   Ref size:   %d  %d\n\n",
		   piFmSize[0], piFmSize[1],
		   m_aiRefSize[0], m_aiRefSize[1]);
		return false;
	}
	//-----------------
	float* pfAugGain = mAugmentRef(m_pfGain, iFactX);
	if(m_pfGain != 0L) cudaFreeHost(m_pfGain);
	m_pfGain = pfAugGain;
	//-----------------
	float* pfAugDark = mAugmentRef(m_pfDark, iFactX);
	if(m_pfDark != 0L) cudaFreeHost(m_pfDark);
	m_pfDark = pfAugDark;
	//-----------------
	m_aiRefSize[0] *= iFactX;
	m_aiRefSize[1] *= iFactX;
	m_aiDarkSize[0] *= iFactX;
	m_aiDarkSize[1] *= iFactY;
	m_bAugmented = true;
	//-----------------
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
	MrcUtil::G90Rotate2D aRot2D;
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
	MrcUtil::GFlip2D aGFlip2D;
	if(iFlip == 1) aGFlip2D.Vertical(gfRef, bGpu, piRefSize);
	else if(iFlip == 2) aGFlip2D.Horizontal(gfRef, bGpu, piRefSize);
}

void CLoadRefs::mInverse(float* gfRef, int* piRefSize, int iInverse)
{
	if(iInverse == 0) return;
	//-----------------------
	MrcUtil::GInverse2D aGInverse2D;
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
	MrcUtil::GAugmentRef aGAugmentRef;
	aGAugmentRef.DoIt(pfRef, m_aiRefSize, pfAugRef, aiAugSize);
	return pfAugRef;
}

