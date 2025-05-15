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
	m_pFindCtf2D = 0L;
}

CFindCtfMain::~CFindCtfMain(void)
{
	if(m_pFindCtf2D != 0L) delete m_pFindCtf2D;
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
	float* gfBuf, DU::CDataPackage* pPackage
)
{	if(!CFindCtfMain::m_bEstCtf) return;
	//---------------------------
	CRescaleImage* pRescaleImg = new CRescaleImage;
	pRescaleImg->DoIt(gfImg, piImgSize, pPackage);
	float* gfPadImgN = pRescaleImg->GetImage();
	m_fPixSizeN = pRescaleImg->m_fPixSizeN;
	//---------------------------
	m_pPackage = pPackage;
	m_pFindCtf2D = new CFindCtf2D;
	//---------------------------
	mGenSpectrums(gfPadImgN, pRescaleImg->m_aiPadSizeN, true, gfBuf);
	//---------------------------
	mFindCTF();
	mEmbedCTF();
	mAddSpectrumToPackage();
	//---------------------------
	bool bAngstrom = true, bDegree = true;
	CCtfParam* pResParam = m_pFindCtf2D->GetResult();
	printf("CTF estimate\n");
	printf("  Df_max [A]   Df_min [A]  Azimuth [d]  "
	   "Phase [d]    Score       Res\n");
	printf("  %9.2f    %9.2f  %8.2f    %7.2f    %9.4f  %7.2f\n",
	   pResParam->GetDfMax(bAngstrom),
	   pResParam->GetDfMin(bAngstrom),
	   pResParam->GetAstAng(bDegree),
	   pResParam->GetExtPhase(bDegree),
	   pResParam->m_fScore,
	   pResParam->m_fCtfRes);
	printf("\n");
	//-----------------
	if(m_pFindCtf2D != 0L) delete m_pFindCtf2D;
	m_pFindCtf2D = 0L;
}

//------------------------------------------------------------
// 1. calculate averged power spectrum over a set of tiles,
// 2. and remove its background for CTF estimation,
// 3. and generate full spectrum.
// 4. m_gfFullSpect is padded with size of [iTileSize + 2,
//    iTileSize].
//------------------------------------------------------------
void CFindCtfMain::mGenSpectrums
(	float* gfImg, int* piImgSize, 
	bool bPadded, float* gfBuf
)
{	CGenAvgSpectrum genAvgSpect;
	genAvgSpect.SetSizes(piImgSize, bPadded, 512);
	int iSpectPixels = genAvgSpect.GetSpectPixels();
	m_gfAvgSpect = gfBuf;
	m_gfFullSpect = m_gfAvgSpect + iSpectPixels;
	m_gfExtBuf = m_gfFullSpect + iSpectPixels * 2;
	//-----------------
	m_aiSpectSize[0] = genAvgSpect.m_aiSpectSize[0];
	m_aiSpectSize[1] = genAvgSpect.m_aiSpectSize[1];
	//-----------------
        genAvgSpect.DoIt(gfImg, m_gfExtBuf, m_gfAvgSpect, m_gfFullSpect);
}

void CFindCtfMain::mFindCTF(void)
{
	CInput* pInput = CInput::GetInstance();
	CCtfParam ctfParam;
	ctfParam.Setup(pInput->m_iKv, pInput->m_fCs,
	   pInput->m_fAmpCont, m_fPixSizeN);
	//---------------------------
	float fDfRange = 40000.0f * m_fPixSizeN * m_fPixSizeN;
	float fPhaseRange = fminf(pInput->m_fExtPhase * 2.0f, 180.0f);
	//---------------------------
	m_pFindCtf2D->Setup1(m_aiSpectSize);
	m_pFindCtf2D->Setup2(&ctfParam);
	m_pFindCtf2D->SetDfRange(fDfRange);
	m_pFindCtf2D->SetAstRange(0.10f);
	m_pFindCtf2D->SetAngRange(180.0f);
	m_pFindCtf2D->SetPhaseRange(fPhaseRange);
	m_pFindCtf2D->DoIt(m_gfAvgSpect);
	//---------------------------
	CCtfParam* pResParam = m_pFindCtf2D->GetResult();
        m_pPackage->SetCtfParam(pResParam);
}

void CFindCtfMain::mEmbedCTF(void)
{
	Util::GCalcMoment2D calcMoment;
        bool bSync = true, bPadded = true;
        calcMoment.SetSize(m_aiSpectSize, !bPadded);
        float fMean = calcMoment.DoIt(m_gfAvgSpect, 1, bSync);
        float fStd = calcMoment.DoIt(m_gfAvgSpect, 2, bSync);
        fStd = fStd - fMean * fMean;
        if(fStd < 0) fStd = 0.0f;
        else fStd = sqrt(fStd);
	float fGain = fStd * 1.5f;
	//-----------------
	float afResRange[2] = {0.0f};
	m_pFindCtf2D->GetResRange(afResRange);
	//-----------------
	CCtfParam* pResParam = m_pFindCtf2D->GetResult();
	float fPixelSize = pResParam->GetPixSize();
        float fMinFreq = fPixelSize / afResRange[0];
        float fMaxFreq = fPixelSize / afResRange[1];
        //-----------------
        GCalcCTF2D calcCtf2D;
        calcCtf2D.DoIt(pResParam, m_gfExtBuf, m_aiSpectSize);
	//-----------------
        calcCtf2D.EmbedCtf(m_gfExtBuf, m_aiSpectSize,
	   fMinFreq, fMaxFreq, fMean, fGain, 
	   m_gfFullSpect, true);
}

void CFindCtfMain::mAddSpectrumToPackage(void)
{
	int aiFullSize[] = {0, m_aiSpectSize[1]};
	aiFullSize[0] = (m_aiSpectSize[0] - 1) * 2;
	//-----------------
	DU::CMrcStack* pCtfStack = new DU::CMrcStack;
        pCtfStack->Create(2, aiFullSize, 1);
        float* pfCtfImg = (float*)pCtfStack->GetFrame(0);
	//-----------------
	Util::CPad2D pad2D;
	int aiPadSize[] = {m_aiSpectSize[0] * 2, m_aiSpectSize[1]};
	pad2D.Unpad(m_gfFullSpect, aiPadSize, pfCtfImg);
        //-----------------
	m_pPackage->SetCtfStack(pCtfStack);
}
