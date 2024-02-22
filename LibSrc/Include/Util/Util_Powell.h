#pragma once

class Util_Powell
{
public:
	Util_Powell(void);
	virtual ~Util_Powell(void);
	virtual float Eval(float* pfPoint); // must be overriden
	void Clean(void);
	void Setup(int iDim, int iIterations, float fTol);
	float DoIt
	( float* pfInitPoint,
	  float* pfSearchRange,
	  int iNumSteps
	);
	int m_iDim;
	float* m_pfInitPoint;
	float* m_pfBestPoint;
	float m_fInitVal;
	float m_fBestVal;
private:
	float mDoIt(void);
	float mLineMinimize(float* pfPoint, float* pfVector);
	void mCalcNewPoint
	( float* pfOldPoint, 
	  float* pfVector,
	  float fStride,
	  float* pfNewPoint
	);
	void mNormVector(float* pfVector);
	void mFindAllowableRange
	( float* pfStartPoint,
	  float* pfVector,
	  float* pfRange
	);
	int m_iIterations;
	int m_iNumSteps;
	float m_fTol;
	float* m_pfPointMin;
	float* m_pfPointMax;
	float* m_pfVectors;
	float m_fTiny;
};

