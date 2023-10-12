#include "CDetectMain.h"
#include <Mrcfile/CMrcFileInc.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::BadPixel;

CTemplate::CTemplate(void)
{
}

CTemplate::~CTemplate(void)
{
}

void CTemplate::Create(int* piSize, float* pfMod)
{
	m_aiSize[0] = piSize[0];
	m_aiSize[1] = piSize[1];
	int iSize = m_aiSize[0] * m_aiSize[1];
	//------------------------------------
	int iEndX = m_aiSize[0] - 1;
	int iEndY = m_aiSize[1] - 1;
	//--------------------------
	for(int i=0; i<iSize; i++)
	{	int x = i % m_aiSize[0];
		int y = i / m_aiSize[0];
		if(x == 0 || x == iEndX) pfMod[i] = -1.0f;
		else if(y == 0 || y == iEndY) pfMod[i] = -1.0f;
		else pfMod[i] = 1.0f;
	}
	//---------------------------
	mNormalize(pfMod);
}

void CTemplate::mNormalize(float* pfTemplate)
{
	double dMean = 0, dStd = 0;
	int iSize = m_aiSize[0] * m_aiSize[1];
	for(int i=0; i<iSize; i++)
	{	dMean += pfTemplate[i];
		dStd += (pfTemplate[i] * pfTemplate[i]);
	}
	float fMean = (float)(dMean / iSize);
	dStd = dStd / iSize - fMean * fMean;
	float fStd = (dStd <= 0) ? 0.0f : (float)sqrt(dStd);
	fStd += (float)(1e-30);
	//---------------------
	for(int i=0; i<iSize; i++)
	{	pfTemplate[i] = (pfTemplate[i] - fMean) / fStd;
	}	
}

