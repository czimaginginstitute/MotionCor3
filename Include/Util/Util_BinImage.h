#pragma once

class Util_BinImage
{
public:

	Util_BinImage(void);
	
	~Util_BinImage(void);

	void SetSize(int iImageX, int iImageY);

	void SetImage(void* pvImage, int iMode);

	void* DoIt(int iBinX, int iBinY);

	int m_iSize[2];

private:

	void* mCalculate(unsigned char* pcImage);

	void* mCalculate(short* psImage);

	void* mCalculate(float* pfImage);

	void* mCalculate(unsigned short* psImage);

	unsigned char mCalcMean(unsigned char* pcImage, int iX, int iY); 

	short mCalcMean(short* psImage, int iX, int iY);

	float mCalcMean(float* pfImage, int iX, int iY);

	unsigned short mCalcMean(unsigned short* psImage, int iX, int iY); 

	int m_iBin[2];

	int m_iImage[2];

	double m_dFactor;

	int m_iMode;

	void* m_pvImage;
};
