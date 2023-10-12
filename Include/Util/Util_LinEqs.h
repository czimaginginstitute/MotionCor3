#pragma once

class Util_LinEqs
{
public:
	Util_LinEqs(void);
	~Util_LinEqs(void);
	bool DoIt
	( float* pfCoeff,
	  float* pfVals,
	  int iDim
	);
	void Test(void);
private:
	bool mFindPivot
	( float* pfCoeff,
	  int iPiv
	);
	void mSwapRows
	( float* pfCoeff,
	  float* pfVals,
	  int iPiv
	);
	void mNormalize
	( float* pfCoeff,
	  float* pfVals,
	  int iPiv
	);
	void mReduceRows
	( float* pfCoeff,
	  float* pfVals,
	  int iPiv
	);
	void mClean
	( void
	);
	int m_iDim;
	int* m_piColIdx; // column index
	int* m_piRowIdx; // row index
	int* m_piPivots; // Num pivots at each column
	
};
