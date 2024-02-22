#include "Util_Powell.h"
#include <math.h>
#include <memory.h>
#include <stdio.h>

Util_Powell::Util_Powell(void)
{
	m_fTiny = (float)1e-25;
	m_iIterations = 200;
	m_fTol = 0.001f;
	//--------------
	m_pfBestPoint = 0L;
	m_pfInitPoint = 0L;
	m_pfPointMin = 0L;
	m_pfPointMax = 0L;
	m_pfVectors = 0L;
}

Util_Powell::~Util_Powell(void)
{
	this->Clean();
}

float Util_Powell::Eval(float* pfPoint)
{
	return 0.0f;
}

void Util_Powell::Clean(void)
{
	if(m_pfBestPoint != 0L) delete[] m_pfBestPoint;
	if(m_pfInitPoint != 0L) delete[] m_pfInitPoint;
	if(m_pfPointMin != 0L) delete[] m_pfPointMin;
	if(m_pfPointMax != 0L) delete[] m_pfPointMax;
	if(m_pfVectors != 0L) delete[] m_pfVectors;
	m_pfBestPoint = 0L;
	m_pfInitPoint = 0L;
	m_pfPointMin = 0L;
	m_pfPointMax = 0L;
	m_pfVectors = 0L;
}

void Util_Powell::Setup(int iDim, int iIterations, float fTol)
{
	this->Clean();
	//------------
	m_iDim = iDim;
	m_pfInitPoint = new float[m_iDim];
        m_pfBestPoint = new float[m_iDim];
        m_pfPointMin = new float[m_iDim];
        m_pfPointMax = new float[m_iDim];
	m_pfVectors = new float[m_iDim * m_iDim];
	//---------------------------------------
	m_iIterations = iIterations;
	m_fTol = fTol;
}

float Util_Powell::DoIt
(	float* pfInitPoint,
	float* pfSearchRange,
	int iNumSteps
)
{	m_iNumSteps = iNumSteps;
	memcpy(m_pfInitPoint, pfInitPoint, sizeof(float) * m_iDim);
	//---------------------------------------------------------
	for(int i=0; i<m_iDim; i++)
	{	m_pfPointMax[i] = m_pfInitPoint[i] + pfSearchRange[i] * 0.5f;
		m_pfPointMin[i] = m_pfInitPoint[i] - pfSearchRange[i] * 0.5f;
	}
	//-------------------------------------------------------------------
	memset(m_pfVectors, 0, sizeof(float) * m_iDim * m_iDim);
	for(int i=0; i<m_iDim; i++)
	{	float* pfVector = m_pfVectors + i * m_iDim;
		pfVector[i] = pfSearchRange[i];
	}
	m_fBestVal = mDoIt();
	return m_fBestVal;
}

float Util_Powell::mDoIt(void)
{
	float* pfPoint0 = new float[m_iDim]; // Point 0
	float* pfPointN = m_pfBestPoint;     // Point N - last point
	float* pfPointE = new float[m_iDim]; // extrapolated point
	float* pfVector = new float[m_iDim];
	//----------------------------------
	int iVectBytes = sizeof(float) * m_iDim;
	m_fBestVal = this->Eval(m_pfInitPoint);
	//-------------------------------------
	memcpy(pfPoint0, m_pfInitPoint, iVectBytes);
	memcpy(pfPointN, m_pfInitPoint, iVectBytes);
	float fValN = m_fBestVal;
	//-----------------------
	int iIter = 0;
	printf("Conjugate gradient refinement\n");
	for(iIter=0; iIter<m_iIterations; iIter++)
	{	float fMaxDrop = 0.0f;
		int iMaxVector = 0;
		float fLastBest = m_fBestVal;
		printf("...... Iter: %4d  Score: %.6e\n", iIter, m_fBestVal); 
		//-----------------------------------------------------------
		for(int i=0; i<m_iDim; i++) // find Point N
		{	memcpy(pfVector, m_pfVectors+i*m_iDim, iVectBytes);
			float fLastBest1 = m_fBestVal;
			mLineMinimize(pfPointN, pfVector);
			float fDrop = fLastBest1 - m_fBestVal;
			//------------------------------------
			if(fDrop <= fMaxDrop) continue;
			fMaxDrop = fDrop;
			iMaxVector = i;
		}
		float fErr = 2.0f * (fLastBest - m_fBestVal);
		double dTol = m_fTol * (fabs(fLastBest) + fabs(m_fBestVal));
		if(fErr <= dTol) break;
		//--------------------------------------------------------
		// 1. Construct a vector that is the averaged direction of
		//    the point movement. Unew = PointN - Point0.
		// 2. Extrapolate PointN along this direction to PointE.
		// 3. Replace Point0 with PointN for next iteration.
		//--------------------------------------------------------
		float* pfNewVect = pfVector;
		for(int i=0; i<m_iDim; i++)
		{	pfNewVect[i] = pfPointN[i] - pfPoint0[i];
			pfPointE[i] = pfPointN[i] + pfNewVect[i];
		}
		memcpy(pfPoint0, pfPointN, iVectBytes);
		//---------------------------------------
		float fValE = this->Eval(pfPointE);
		if(fValE >= m_fBestVal) continue; // new vector bad, discard
		else m_fBestVal = fValE;
		//-------------------------------------------------------------
		// 1. The following checks if it is worth of keeping the
		//    new direction PointN - Point0.
		// 2. If fValE is close to fLastBest, it means the drop along
		//    the new direction is not significant. We should throw
		//    away this new direction. 
		// 3. When fValE is close to fLastBest, fDelta2 is close to 0.
		//    fMaxDrop * fDelta2 * fDelt2 is then close to zero. t 
		//    is then likely positive. This results in discarding the 
		//    new direction.  
		//-------------------------------------------------------------    
		float fDelta1 = fLastBest - m_fBestVal - fMaxDrop;
		float fDelta2 = fLastBest - fValE; 
		float t = 2.0f * (fLastBest - 2.0f * m_fBestVal + fValE) 
			* fDelta1 * fDelta1 - fMaxDrop * fDelta2 * fDelta2;
		if(t >= 0) continue; 
		//------------------------------------------------------------
		// 1. The new direction is good. Find the minimum along this
		//    direction. Replace Umax with Un and then replace Un
		//    with the new direction (PointN - Point0).
		//------------------------------------------------------------
		mLineMinimize(pfPointN, pfNewVect);
		float* pfVectN = m_pfVectors + (m_iDim - 1) * m_iDim;
		float* pfVectMax = m_pfVectors + iMaxVector * m_iDim;
		memcpy(pfVectMax, pfVectN, iVectBytes);
		memcpy(pfVectN, pfNewVect, iVectBytes);
	}
	printf("Total Iters: %4d  Score: %.6e\n\n", iIter+1, m_fBestVal);
	//---------------------------------------------------------------
	if(pfPoint0 != 0L) delete[] pfPoint0;
	if(pfPointE != 0L) delete[] pfPointE;
	if(pfVector != 0L) delete[] pfVector;
	return m_fBestVal;
}

float Util_Powell::mLineMinimize(float* pfPoint, float* pfVector)
{
	float afRange[2] = {0.0f};
	mNormVector(pfVector); // vector normalized to unit vector
	mFindAllowableRange(pfPoint, pfVector, afRange);
	//----------------------------------------------
	int iCent = m_iNumSteps / 2;
	int iBestStep = -1;
	//-----------------
	float* pfNewPoint = new float[m_iDim];
	float* pfVals = new float[m_iNumSteps];
	float fStep = (afRange[1] - afRange[0]) / m_iNumSteps;
	//----------------------------------------------------
	for(int i=0; i<m_iNumSteps; i++)
	{	float fStride = fStep * (i - iCent);
		mCalcNewPoint(pfPoint, pfVector, fStride, pfNewPoint);
		pfVals[i] = this->Eval(pfNewPoint);
		/*
		printf("Line min: stride %d, val: %.8e, min: %.8e\n",
			i, pfVals[i], m_fBestVal);
		*/
		if(pfVals[i] >= m_fBestVal) continue;
		m_fBestVal = pfVals[i];
		iBestStep = i;
	}
	if(pfVals != 0L) delete[] pfVals;
	if(iBestStep < 0) return m_fBestVal;
	//----------------------------------
	float fStride = fStep * (iBestStep - iCent);
	mCalcNewPoint(pfPoint, pfVector, fStride, pfPoint);
	if(iBestStep == 0 || iBestStep == (m_iNumSteps - 1))
	{	delete[] pfNewPoint;
		return m_fBestVal;
	}
	//--------------------------------------------------------------
	// Numerical Recipes: Parabolic Interpolation and Brent's Method
	//--------------------------------------------------------------
	float fa = pfVals[iBestStep - 1];
	float fb = pfVals[iBestStep];
	float fc = pfVals[iBestStep + 1];
	float fFract = -0.5f * (fc - fa) / (fa - 2.0f * fb + fc + m_fTiny);
	if(fFract <= -1 || fFract >= 1 || fFract == 0)
	{	if(pfNewPoint != 0L) delete[] pfNewPoint;
		return m_fBestVal;
	}
	//------------------------ 
	fStride = fStep * fFract;
	mCalcNewPoint(pfPoint, pfVector, fStride, pfNewPoint);
	float fValInt = this->Eval(pfNewPoint);
	if(fValInt < m_fBestVal)
	{	m_fBestVal = fValInt;
		memcpy(pfPoint, pfNewPoint, sizeof(float) * m_iDim);
	}
	if(pfNewPoint != 0L) delete[] pfNewPoint;
	return m_fBestVal;
}

void Util_Powell::mCalcNewPoint
(	float* pfOldPoint, // starting point
	float* pfVector,   // must be unit vector   
	float fStride,     // movement along pfVector
	float* pfNewPoint  // new point at the step 
)
{	for(int i=0; i<m_iDim; i++)
	{	pfNewPoint[i] = pfOldPoint[i] + fStride * pfVector[i];
		//printf(".... %.4f \n", pfNewPoint[i]);
	}
	//printf("\n");
}

//-------------------------------------------------------------------
// 1. pfRange stores the maximum allowed strides along pfVector
//    starting from the pfStartPoint. The allowed strides ensure
//    the line minimization will not go beyond allowed searching
//    range defined by m_pfPointMin and m_pfPointMax.
// 2. Note that pfVector doesn't have to be unit vector. pfRange
//    determines how far minimum will be searched along pfVector.
//-------------------------------------------------------------------    
void Util_Powell::mFindAllowableRange
(	float* pfStartPoint,
	float* pfVector, 
	float* pfRange
)
{	pfRange[0] = (float)1e30;
	pfRange[1] = (float)1e30;
	//-----------------------
	for(int i=0; i<m_iDim; i++)
	{	if(pfVector[i] == 0) continue;
		float fLeft = (pfStartPoint[i] - m_pfPointMin[i]) 
			/ pfVector[i];
		float fRight = (m_pfPointMax[i] - pfStartPoint[i]) 
			/ pfVector[i];
		if(fLeft < pfRange[0]) pfRange[0] = fLeft;
		if(fRight < pfRange[1]) pfRange[1] = fRight;
	}
	pfRange[0] = -pfRange[0];
}

void Util_Powell::mNormVector(float* pfVector)
{
	float fMag = 0.0;
	for(int i=0; i<m_iDim; i++)
	{	fMag += (pfVector[0] * pfVector[0]);
	}
	fMag = (float)(sqrtf(fMag) + 1e-30);
	for(int i=0; i<m_iDim; i++)
	{	pfVector[i] /= fMag;
	}
}
