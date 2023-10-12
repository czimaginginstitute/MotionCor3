#include "CAlignInc.h"
#include "../Util/CUtilInc.h"
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::Align;

CStackShift::CStackShift(void)
{
	m_pfShiftXs = 0L;
	m_iNumFrames = 0;
	memset(m_afCenter, 0, sizeof(m_afCenter));
}

CStackShift::~CStackShift(void)
{
	if(m_pfShiftXs != 0L) delete[] m_pfShiftXs;
}

void CStackShift::Setup(int iNumFrames)
{
	this->Clear();
	if(iNumFrames <= 0) return;
	//-------------------------
	m_iNumFrames = iNumFrames;
	int iElems = m_iNumFrames * 2;
	m_pfShiftXs = new float[iElems];
	m_pfShiftYs = m_pfShiftXs + m_iNumFrames;
	memset(m_pfShiftXs, 0, sizeof(float) * iElems);
}

void CStackShift::SetCenter(int* piStart, int* piSize)
{
	m_afCenter[0] = piStart[0] + piSize[0] * 0.5f;
	m_afCenter[1] = piStart[1] + piSize[1] * 0.5f;
}

void CStackShift::SetCenter(float fCentX, float fCentY)
{
	m_afCenter[0] = fCentX;
	m_afCenter[1] = fCentY;
}

void CStackShift::Clear(void)
{
	if(m_pfShiftXs != 0L) delete[] m_pfShiftXs;
	m_pfShiftXs = 0L;
	m_pfShiftYs = 0L;
	m_iNumFrames = 0;
}

void CStackShift::Reset(void)
{
	if(m_pfShiftXs == 0L) return;
	int iBytes = m_iNumFrames * 2 * sizeof(float);
	memset(m_pfShiftXs, 0, iBytes);
}

void CStackShift::SetShift(int iFrame, float* pfShift)
{
	m_pfShiftXs[iFrame] = pfShift[0];
	m_pfShiftYs[iFrame] = pfShift[1];
}

void CStackShift::SetShift(CStackShift* pSrcShift)
{
	if(pSrcShift == 0L) return;
	int iBytes = m_iNumFrames * 2 * sizeof(float);
	memcpy(m_pfShiftXs, pSrcShift->m_pfShiftXs, iBytes);
}

void CStackShift::AddShift(int iFrame, float* pfShift)
{
        m_pfShiftXs[iFrame] += pfShift[0];
        m_pfShiftYs[iFrame] += pfShift[1];
}

void CStackShift::AddShift(CStackShift* pDeltaShift)
{
	if(pDeltaShift == 0L) return;
	for(int i=0; i<m_iNumFrames; i++)
	{	m_pfShiftXs[i] += pDeltaShift->m_pfShiftXs[i];
		m_pfShiftYs[i] += pDeltaShift->m_pfShiftYs[i];
	}
}

void CStackShift::MakeRelative(int iRefFrame)
{
	float fRefShiftX = m_pfShiftXs[iRefFrame];
	float fRefShiftY = m_pfShiftYs[iRefFrame];
	//----------------------------------------
	for(int i=0; i<m_iNumFrames; i++)
	{	m_pfShiftXs[i] -= fRefShiftX;
		m_pfShiftYs[i] -= fRefShiftY;
	}
}

void CStackShift::Multiply(float fFactX, float fFactY)
{
	for(int i=0; i<m_iNumFrames; i++)
	{	m_pfShiftXs[i] *= fFactX;
		m_pfShiftYs[i] *= fFactY;
	}
}

void CStackShift::TruncateDecimal(void)
{
	for(int i=0; i<m_iNumFrames; i++)
	{	m_pfShiftXs[i] = 0.01f * (int)(m_pfShiftXs[i] * 100);
		m_pfShiftYs[i] = 0.01f * (int)(m_pfShiftYs[i] * 100);
	}
}

void CStackShift::GetShift(int iFrame, float* pfShift, float fFact)
{
	pfShift[0] = m_pfShiftXs[iFrame] * fFact;
	pfShift[1] = m_pfShiftYs[iFrame] * fFact;
}

void CStackShift::GetRelativeShift
(	int iFrame, 
	float* pfShift, 
	int iRefFrame
)
{	float afRefShift[2] = {0.0f};
	this->GetShift(iFrame, pfShift);
	this->GetShift(iRefFrame, afRefShift);
	pfShift[0] -= afRefShift[0];
	pfShift[1] -= afRefShift[1];
}

float* CStackShift::GetShifts(void)
{
	return m_pfShiftXs;
}

void CStackShift::GetCenter(float* pfLoc)
{
	pfLoc[0] = m_afCenter[0];
	pfLoc[1] = m_afCenter[1];
}

CStackShift* CStackShift::GetCopy(void)
{
	CStackShift* pStackShift = new CStackShift;
	pStackShift->Setup(m_iNumFrames);
	pStackShift->SetCenter(m_afCenter[0], m_afCenter[1]);
	int iBytes = m_iNumFrames * 2 * sizeof(float);
	if(iBytes <= 0) return pStackShift;
	//---------------------------------
	memcpy(pStackShift->m_pfShiftXs, m_pfShiftXs, iBytes);
	return pStackShift;
}

int CStackShift::GetCentralFrame(void)
{
	int iCent = m_iNumFrames / 2;
	return iCent;
}

void CStackShift::RemoveSpikes(bool bSingle)
{
	Util::CRemoveSpikes1D removeSpikes;
	removeSpikes.SetDataSize(m_iNumFrames);
	removeSpikes.DoIt(m_pfShiftXs, m_pfShiftYs, bSingle);
}

void CStackShift::DisplayShift(const char* pcHeader, int iRow)
{
	if(pcHeader != 0L) printf("%s\n", pcHeader);
	float afRef[2] = {0.0f};
	if(iRow >=0 ) this->GetShift(iRow, afRef);
	for(int i=0; i<m_iNumFrames; i++)
	{	printf("...... Frame (%3d) shift: %9.3f  %9.3f\n", i+1,
		   m_pfShiftXs[i] - afRef[0], m_pfShiftYs[i] - afRef[1]);
	}
	printf("\n");
}

void CStackShift::Smooth(float fWeight)
{
	if(fWeight >= 1) return;
	//----------------------
	int iLast = m_iNumFrames - 1;
	float* pfSmoothedX = new float[m_iNumFrames * 2];
	float* pfSmoothedY = pfSmoothedX + m_iNumFrames;
	memcpy(pfSmoothedX, m_pfShiftXs, sizeof(float) * m_iNumFrames * 2);
	//-----------------------------------------------------------------
	float fW2 = 0.5f * (1.0f - fWeight);
	for(int i=1; i<iLast; i++)
	{	pfSmoothedX[i] = fWeight * m_pfShiftXs[i] + fW2
		   * (m_pfShiftXs[i-1] + m_pfShiftXs[i+1]);
		pfSmoothedY[i] = fWeight * m_pfShiftYs[i] + fW2
		   * (m_pfShiftYs[i-1] + m_pfShiftYs[i+1]);
	}
	memcpy(m_pfShiftXs, pfSmoothedX, sizeof(float) * m_iNumFrames * 2);
	delete[] pfSmoothedX;
}	
