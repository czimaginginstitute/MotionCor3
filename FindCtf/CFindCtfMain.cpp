#include "CFindCtfInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2;
using namespace MotionCor2::FindCtf;
namespace DU = MotionCor2::DataUtil;

bool CFindCtfMain::m_bEstCtf = true;

CFindCtfMain::CFindCtfMain(void)
{
}

CFindCtfMain::~CFindCtfMain(void)
{
}

bool CFindCtfMain::bEstimate(void)
{
	CInput* pInput = CInput::GetInstance();
	m_bEstCtf = true;
	if(pInput->m_fCs <= 0.001) m_bEstCtf = false;
	else if(pInput->m_iKv == 0) m_bEstCtf = false;
	else if(pInput->m_fPixelSize == 0) m_bEstCtf = false;
	if(m_bEstCtf) return true;
	//------------------------
	printf("Skip CTF estimation. Need the following parameters.\n");
	printf("High tension: %d\n", pInput->m_iKv);
	printf("Cs value:     %f\n", pInput->m_fCs);
	printf("Pixel size:   %f\n\n", pInput->m_fPixelSize);
	return false;	
}

void CFindCtfMain::DoIt
(	float* gfImg, int* piImgSize, bool bPadded,
	float* gfBuf,
	DU::CDataPackage* pPackage
)
{	if(!CFindCtfMain::m_bEstCtf) return;
	//------------------------------------------------------------
	// calculate averged power spectrum over a set of tiles.
	//------------------------------------------------------------
	CGenAvgSpectrum genAvgSpect;
	genAvgSpect.SetSizes(piImgSize, bPadded, 512);
	int iSpectPixels = genAvgSpect.GetSpectPixels();
	float* gfAvgSpect = gfBuf;
	float* gfFullSpect = gfAvgSpect + iSpectPixels;
	float* gfExtBuf = gfFullSpect + iSpectPixels * 2;
	genAvgSpect.DoIt(gfImg, gfExtBuf, gfAvgSpect);
	//------------------------------------------------------------
	// estimate CTF on the averaged power spectrum.
	//------------------------------------------------------------
	CInput* pInput = CInput::GetInstance();
	float fPixSize = pPackage->m_pAlnSums->m_fPixSize;
	CCtfParam ctfParam;
	ctfParam.Setup(pInput->m_iKv, pInput->m_fCs, 
	   pInput->m_fAmpCont, fPixSize);
	//-----------------
	float fDfRange = 40000.0f * fPixSize * fPixSize;
	float fPhaseRange = fminf(pInput->m_fExtPhase * 2.0f, 180.0f);
	//-----------------
	CFindCtf2D* pFindCtf2D = new CFindCtf2D;
	pFindCtf2D->Setup1(genAvgSpect.m_aiSpectSize);
	pFindCtf2D->Setup2(&ctfParam);
	pFindCtf2D->SetDfRange(fDfRange);
	pFindCtf2D->SetAstRange(0.10f);
	pFindCtf2D->SetAngRange(180.0f);
	pFindCtf2D->SetPhaseRange(fPhaseRange);
	pFindCtf2D->DoIt(gfAvgSpect);
	//-----------------
	CCtfParam* pResParam = pFindCtf2D->GetResult();
	pPackage->SetCtfParam(pResParam);
	float afResRange[2] = {0.0f};
	pFindCtf2D->GetResRange(afResRange);
	delete pFindCtf2D;
	//------------------------------------------------------------
	// generate full spectrum with CTF embedded for diagnosis.
	//------------------------------------------------------------
	CFullSpectrum fullSpectrum;
	fullSpectrum.Create(gfAvgSpect, gfExtBuf, 
	   genAvgSpect.m_aiSpectSize, pResParam, 
	   afResRange, gfFullSpect);
	//-----------------
	DU::CMrcStack* pCtfStack = new DU::CMrcStack;
        pCtfStack->Create(2, fullSpectrum.m_aiFullSize, 1);
	float* pfCtfImg = (float*)pCtfStack->GetFrame(0);
	fullSpectrum.ToHost(pfCtfImg);
	pPackage->SetCtfStack(pCtfStack);
	//-----------------
	bool bAngstrom = true, bDegree = true;
	printf("CTF estimate\n");
	printf("  Df_max [A]   Df_min [A]  Azimuth [d]  "
	   "Phase [d]    Score\n");
	printf("  %9.2f    %9.2f  %8.2f    %7.2f    %9.4f\n",
	   pResParam->GetDfMax(bAngstrom),
	   pResParam->GetDfMin(bAngstrom),
	   pResParam->GetAstAng(bDegree),
	   pResParam->GetExtPhase(bDegree),
	   pResParam->m_fScore);
	printf("\n");
}

