#include "CFindCtfInc.h"

using namespace MotionCor2::FindCtf;

float CFindCtfHelp::CalcAstRatio(float fDfMin, float fDfMax)
{
	float fDiff = (fDfMax - fDfMin) * 0.5f;
	float fMean = (fDfMax + fDfMin) * 0.5f;
	if(fMean <= 0.0f) return 0.0f;
	else return (fDiff / fMean);
}

float CFindCtfHelp::CalcDfMin(float fDfMean, float fAstRatio)
{
	float fDfMin = fDfMean * (1.0f - fAstRatio);
	return fDfMin;
}

float CFindCtfHelp::CalcDfMax(float fDfMean, float fAstRatio)
{
	float fDfMax = fDfMean * (1.0f + fAstRatio);
	return fDfMax;
}

