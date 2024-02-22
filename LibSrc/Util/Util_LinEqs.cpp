#include "Util_LinEqs.h"
#include <memory.h>
#include <math.h>
#include <stdio.h>

Util_LinEqs::Util_LinEqs(void)
{
	m_iDim = 0;
	m_piColIdx = 0L;
	m_piRowIdx = 0L;
	m_piPivots = 0L;
}

Util_LinEqs::~Util_LinEqs(void)
{
	mClean();
}

bool Util_LinEqs::DoIt
(	float* pfCoeff,
	float* pfVals,
	int iDim
)
{	mClean();
	m_iDim = iDim;
	m_piColIdx = new int[m_iDim];
	m_piRowIdx = new int[m_iDim];
	m_piPivots = new int[m_iDim];
	memset(m_piPivots, 0, sizeof(int) * m_iDim);
	//------------------------------------------
	for(int iPiv=0; iPiv<m_iDim; iPiv++)
	{	bool bFound = mFindPivot(pfCoeff, iPiv);
		if(!bFound)
		{	mClean();
			return false;
		}
		mSwapRows(pfCoeff, pfVals, iPiv);
		mNormalize(pfCoeff, pfVals, iPiv);
		mReduceRows(pfCoeff, pfVals, iPiv);
	}
	return true;
}

//=============================================================================
// Search the nth (iPiv) pivotal element in the entire matrix. However, skip
// all rows and all columns where pivotal elements have been identified.
//============================================================================= 
bool Util_LinEqs::mFindPivot(float* pfCoeff, int iPiv)
{
	int iX = 0, iY= 0;
	bool bFound = false;
	float fMax = 0.0f;
	//----------------
	for(int y=0; y<m_iDim; y++)
	{	if(m_piPivots[y] == 1) continue;
		for(int x=0; x<m_iDim; x++)
		{	if(m_piPivots[x] == 1) continue;
			int i = y * m_iDim + x;
			float fCoeff = (float)fabs(pfCoeff[i]);
			if(fCoeff <= fMax) continue;
			fMax = fCoeff;
			iX = x;
			iY = y;
			bFound = true;
		}
	}
	if(!bFound) return false;
	//-----------------------
	m_piPivots[iX] += 1;
	m_piColIdx[iPiv] = iX;
	m_piRowIdx[iPiv] = iY;
	//--------------------
	if(pfCoeff[iY * m_iDim + iX] == 0.0f)
	{	fprintf
		(  stderr, "Util_LinEqs: %s\n",
		   "encounter singular matrix."
		);
		return false;
	}
	return true;
}

//=============================================================================
// Once the pivotal element has been found in a row and a column (iRowPiv,
// iColPiv). swap this row with the row whose index is the same as iColPiv.
// Therefore, the pivotal element is at diagonal (iColPiv, iColPiv).
//============================================================================= 
void Util_LinEqs::mSwapRows
( float* pfCoeff,
  float* pfVals,
  int iPiv
)
{	int iRow1 = m_piColIdx[iPiv];
	int iRow2 = m_piRowIdx[iPiv];
	if(iRow1 == iRow2) return;
	//------------------------
	float* pfBuf = new float[m_iDim];
	float* pfRow1 = pfCoeff + iRow1 * m_iDim;
	float* pfRow2 = pfCoeff + iRow2 * m_iDim;
	//---------------------------------------
	int iBytes = sizeof(float) * m_iDim;
	memcpy(pfBuf, pfRow1, iBytes);
	memcpy(pfRow1, pfRow2, iBytes);
	memcpy(pfRow2, pfBuf, iBytes);
	delete[] pfBuf;
	//-------------
	float fVal = pfVals[iRow1];
	pfVals[iRow1] = pfVals[iRow2];
	pfVals[iRow2] = fVal;
}

void Util_LinEqs::mNormalize
( 	float* pfCoeff,
  	float* pfVals,
  	int iPiv
)
{	int iCol = m_piColIdx[iPiv];
	int iIdx = iCol * m_iDim;
	double dPivInv = 1.0 / (pfCoeff[iIdx + iCol] + 1e-30);
	//----------------------------------------------------
	for(int x=0; x<m_iDim; x++)
	{	int i = iIdx + x;
		double dCoeff = pfCoeff[i] * dPivInv;
		pfCoeff[i] = (float)dCoeff;
	}
	double dVal = pfVals[iCol] * dPivInv;
	pfVals[iCol] = (float)dVal;
}

void Util_LinEqs::mReduceRows
(	float* pfCoeff,
	float* pfVals,
	int iPiv
)
{	int iCol = m_piColIdx[iPiv];
	int iRow = iCol;
	int iIdx = iRow * m_iDim;
	//-----------------------
	for(int y=0; y<m_iDim; y++)
	{	if(y == m_piColIdx[iPiv]) continue;
		int i = y * m_iDim;
		float fDum = pfCoeff[i + iCol];
		for(int x=0; x<m_iDim; x++)
		{	pfCoeff[i+x] -= (pfCoeff[iIdx+x] * fDum);
		}
		pfVals[y] -= (pfVals[iRow] * fDum);
	}
}

void Util_LinEqs::mClean(void)
{
	if(m_piColIdx != 0L) delete[] m_piColIdx;
	if(m_piRowIdx != 0L) delete[] m_piRowIdx;
	if(m_piPivots != 0L) delete[] m_piPivots;
	m_piColIdx = 0L;
	m_piRowIdx = 0L;
	m_piPivots = 0L;
}

void Util_LinEqs::Test(void)
{
	float afCoeff[] = 
	{ 0.2368f, 0.2471f, 0.2568f, 1.2671f,
	  0.1968f, 0.2071f, 1.2168f, 0.2271f,
	  0.1581f, 1.1675f, 0.1768f, 0.1871f,
	  1.1161f, 0.1254f, 0.1397f, 0.1490f
	};
	float afVals[] =
	{ 1.8471f, 1.7471f, 1.6471f, 1.5471f
	};
	float afSol[] = 
	{ 1.0405f, 0.9870f, 0.9350f, 0.8812f
	};
	int iDim = 4;
	//-----------
	this->DoIt(afCoeff, afVals, iDim);
	for(int i=0; i<iDim; i++)
	{	printf("Sol: %8.4f, Ans: %8.4f\n", afVals[i], afSol[i]);
	}
}

