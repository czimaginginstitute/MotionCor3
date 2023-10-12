#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <Util/Util_Time.h>
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CPatchCenters* CPatchCenters::m_pInstance = 0L;

CPatchCenters* CPatchCenters::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CPatchCenters;
	return m_pInstance;
}

void CPatchCenters::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CPatchCenters::CPatchCenters(void)
{
	m_iNumPatches = 0;
	m_piPatStarts = 0L;
}

CPatchCenters::~CPatchCenters(void)
{
	if(m_piPatStarts != 0L) delete[] m_piPatStarts;
	m_piPatStarts = 0L;
}

void CPatchCenters::Calculate(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pXcfBuffer = pBufferPool->GetBuffer(EBuffer::xcf);
	CStackBuffer* pPatBuffer = pBufferPool->GetBuffer(EBuffer::pat);
	m_aiXcfSize[0] = (pXcfBuffer->m_aiCmpSize[0] - 1) * 2;
	m_aiXcfSize[1] = pXcfBuffer->m_aiCmpSize[1];
	//------------------------------------------
	CInput* pInput = CInput::GetInstance();
	m_aiPatSize[0] = m_aiXcfSize[0] / pInput->m_aiNumPatches[0] / 2 * 2;
	m_aiPatSize[1] = m_aiXcfSize[1] / pInput->m_aiNumPatches[1] / 2 * 2;
	//------------------------------------------------------------------
	m_iNumPatches = pInput->m_aiNumPatches[0] * pInput->m_aiNumPatches[1];
	if(m_piPatStarts != 0L) delete[] m_piPatStarts;
	m_piPatStarts = new int[2 * m_iNumPatches];
	//-----------------------------------------
	CDetectFeatures* pDetectFeatures = CDetectFeatures::GetInstance();
	float afLoc[2], afNewLoc[2];
	for(int i=0; i<m_iNumPatches; i++)
	{	int x = i % pInput->m_aiNumPatches[0];
		int y = i / pInput->m_aiNumPatches[0];
		afLoc[0] = (x + 0.5f) * m_aiPatSize[0];
		afLoc[1] = (y + 0.5f) * m_aiPatSize[1];
		//pDetectFeatures->FindNearest(afLoc, m_aiXcfSize, 
		//   m_aiPatSize, afNewLoc);
		pDetectFeatures->GetCenter(i, m_aiXcfSize, afNewLoc);
		//------------------------
		if(i == 0) 
		{	printf("# xcf img size: %d  %d\n", 
			   m_aiXcfSize[0], m_aiXcfSize[1]);
			printf("# CPatchCenters::Calculate()\n");
		}
		printf("%3d %9.2f  %9.2f  %9.2f  %9.2f\n", i,
		   afLoc[0], afLoc[1], afNewLoc[0], afNewLoc[1]);
		//---------------------------------------------------------
		int j =  2 * i;
		m_piPatStarts[j] = mCalcStart(afNewLoc[0], 
		   m_aiPatSize[0], m_aiXcfSize[0]);
		m_piPatStarts[j+1] = mCalcStart(afNewLoc[1], 
		   m_aiPatSize[1], m_aiXcfSize[1]);
	}
}	

void CPatchCenters::GetCenter(int iPatch, int* piCenter)
{
	int i = 2 * iPatch;
	piCenter[0] = m_piPatStarts[i] + m_aiPatSize[0] / 2;
	piCenter[1] = m_piPatStarts[i+1] + m_aiPatSize[1] / 2;
}

void CPatchCenters::GetStart(int iPatch, int* piStart)
{
	int i = 2 * iPatch;
	piStart[0] = m_piPatStarts[i];
	piStart[1] = m_piPatStarts[i+1];
}	

int CPatchCenters::mCalcStart(float fCent, int iPatSize, int iXcfSize)
{
	int iStart = (int)(fCent - iPatSize / 2);
	if(iStart < 0) iStart = 0;
	else if((iStart + iPatSize) > iXcfSize) iStart =  iXcfSize - iPatSize;
	return iStart;
}
	
