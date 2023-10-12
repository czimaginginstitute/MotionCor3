#pragma once
#include <CuUtil/DeviceArray2D.h>
#include <cufft.h>

namespace CuUtilFFT
{

//-------------------------------------------------------------------
// 1. Perform forward and inverse 2D Fourier transform.
// 2. The forward FFT can be done with normalization.
//-------------------------------------------------------------------
class GFFT2D
{
public:
	GFFT2D(void);
	~GFFT2D(void);
	void SetStream(cudaStream_t stream);
	void DestroyPlan(void);
	void CreatePlan(int* piFFTSize, bool bForward);
	void Forward(float* gfPadImg, bool bNorm);
	void Forward(float* gfImg, cufftComplex* gCmp, bool bNorm);
	void Inverse(cufftComplex* gCmp);
	void Inverse(cufftComplex* gCmp, float* gfImg);
	void RemoveAmp(cufftComplex* gCmp, int* piCmpSize);
private:
	void mNormalize(cufftComplex* gCmpImg);
	void mCheckError(cufftResult error, const char* pcFunc);
	int m_aiFFTSize[2];
	cufftType m_cufftType;
	cufftHandle m_cufftPlan;
	cudaStream_t m_aStream;
};

//-------------------------------------------------------------------
// 1. Perform 2D rotation in Fourier space using CUDA texture based
//    bilinear intepolation.
// 2. The input and output size can be different.
//-------------------------------------------------------------------
class GRotFFT2D
{
public:
	GRotFFT2D(void);
	~GRotFFT2D(void);
	void Clean(void);
	void SetFFT
	(  cufftComplex* gCmp,
	   int* piCmpSize,
	   bool bGpu
	);
	cufftComplex* DoIt
	(  float fAngle,
	   int* piCmpSize
	);
	void DoIt
	(  float fAngle,
	   cufftComplex* gOutCmp,
	   int* piCmpSize
	);
private:
	void mShiftCenter(cufftComplex* gCmp, int* piCmpSize);
	cufftComplex* mGetCmpBuf(int* piCmpSize, bool bZero);
	cufftComplex* mHostToDevice(cufftComplex* pCmp, int* piCmpSize);
	CDeviceArray2D* m_pDeviceArray;
	int m_aiInCmpSize[2];
};

//-------------------------------------------------------------------
// 1. Perform 2D translation in Fourier space by shifting phases.
// 2. Center function translates (Nx/2, Ny/2).
//-------------------------------------------------------------------
class GShiftFFT2D
{
public:
	GShiftFFT2D(void);
	~GShiftFFT2D(void);
	void DoIt
	(  cufftComplex* gCmp, 
	   int* piCmpSize,
	   float* pfShift
	);
	void DoIt
	(  cufftComplex* gCmp, 
	   int* piCmpSize,
	   float fShiftX, 
	   float fShiftY
	);
	void Center
	(  cufftComplex* gCmp, 
	   int* piCmpSize
	);
private:
	float m_f2PI;
};

//-------------------------------------------------------------------
// 1. It can crop the 2D Fourier transform if the output size is
//    smaller than the input size.
// 2. It can pad zeroes in the input 2D Fourier transform if the
//    output size is bigger than the input.
//-------------------------------------------------------------------
class GFFTResize2D
{
public:
	GFFTResize2D(void);
	~GFFTResize2D(void);
	void GetNewCmpSize
	(  int* piCmpSize, 
	   float fBinning,
	   int* piNewSize
	);
	void GetNewImgSize
	(  int* piImgSize, 
	   float fBinning,
	   int* piNewSize
	);
	void DoIt
	(  cufftComplex* gCmpIn, 
	   int* piSizeIn,
	   cufftComplex* gCmpOut, 
	   int* piSizeOut
	);
	cufftComplex* DoIt
	(  cufftComplex* gCmpIn, 
	   int* piSizeIn,
	   float fBinning, 
	   int* piSizeOut
	);
private:
	void mCrop
	(  cufftComplex* gCmpIn, int* piSizeIn,
	   cufftComplex* gCmpOut, int* piSizeOut
	);
	void mPad
	(  cufftComplex* gCmpIn, int* piSizeIn,
	   cufftComplex* gCmpOut, int* piSizeOut
	);
};

//-------------------------------------------------------------------
// 1. Perform lowpass filtering using either B-factor or
// 2. Perform lowpass filtering using cutoff frequence (relative).
//-------------------------------------------------------------------
class GLowpass2D
{
public:
	GLowpass2D(void);
	~GLowpass2D(void);
	cufftComplex* DoBFactor
	(  cufftComplex* gCmp, 
	   int* piCmpSize,
	   float fBFactor
	);
	void DoBFactor
	(  cufftComplex* gInCmp,
	   cufftComplex* gOutCmp,
	   int* piCmpSize,
	   float fBFactor
	);
	cufftComplex* DoCutoff
	(  cufftComplex* gCmp,
	   int* piCmpSize,
	   float fCutoff
	);
	void DoCutoff
	(  cufftComplex* gInCmp,
	   cufftComplex* gOutCmp,
	   int* piCmpSize,
	   float fCutoff
	);
};

class GConvolve2D
{
public:
	GConvolve2D(void);
	~GConvolve2D(void);
	void DoIt
	(  cufftComplex* gCmp1,
	   cufftComplex* gCmp2,
	   int* piCmpSize,
	   cufftComplex* gResCmp
	);
};

}
