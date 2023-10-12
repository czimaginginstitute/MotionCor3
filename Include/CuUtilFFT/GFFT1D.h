#pragma once
#include <CuUtil/DeviceArray2D.h>
#include <cufft.h>

namespace CuUtilFFT
{

//-------------------------------------------------------------------
// 1. Perform forward and inverse 1D Fourier transform.
// 2. The forward FFT can be done with normalization.
//-------------------------------------------------------------------
class GFFT1D
{
public:
	GFFT1D(void);
	~GFFT1D(void);
	void DestroyPlan(void);
	void CreatePlan
	( int iFFTSize, 
	  int iNumLines, 
	  bool bForward
	);
	void Forward
	( float* gfPadLines,
	  bool bNorm
	);
	void Inverse
	( cufftComplex* gCmpLines 
	);
private:
	int m_iFFTSize;
	int m_iNumLines;
	cufftType m_cufftType;
	cufftHandle m_cufftPlan;
};

//-------------------------------------------------------------------
// 1. Perform 1D translation in Fourier space by shifting phases.
// 2. Center function translates (Nx/2, Ny/2).
//-------------------------------------------------------------------
class GShiftFFT1D
{
public:
	GShiftFFT1D(void);
	~GShiftFFT1D(void);
	void DoIt
	( cufftComplex* gCmp, 
	  int iCmpSize,
	  float fShift
	);
	void Center
	( cufftComplex* gCmp, 
	  int iCmpSize
	);
private:
	float m_f2PI;
};

//-------------------------------------------------------------------
// 1. Perform lowpass filtering using either B-factor or
// 2. Perform lowpass filtering using cutoff frequence (relative).
//-------------------------------------------------------------------
class GLowpass1D
{
public:
	GLowpass1D(void);
	~GLowpass1D(void);
	cufftComplex* DoBFactor
	( cufftComplex* gCmp, 
	  int iCmpSize,
	  float fBFactor
	);
	void DoBFactor
	( cufftComplex* gInCmp,
	  cufftComplex* gOutCmp,
	  int iCmpSize,
	  float fBFactor
	);
	cufftComplex* DoCutoff
	( cufftComplex* gCmp,
	  int iCmpSize,
	  float fCutoff
	);
	void DoCutoff
	( cufftComplex* gInCmp,
	  cufftComplex* gOutCmp,
	  int iCmpSize,
	  float fCutoff
	);
};

class GConvolve1D
{
public:
	GConvolve1D(void);
	~GConvolve1D(void);
	void DoIt
	( cufftComplex* gCmp1,
	  cufftComplex* gCmp2,
	  int iCmpSize,
	  cufftComplex* gResCmp
	);
};

}
