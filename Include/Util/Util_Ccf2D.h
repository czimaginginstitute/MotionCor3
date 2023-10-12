#pragma once

//=============================================================================
//
// This class is used to calculate linear correlation coefficient between a
// pair of two dimensional images.
//
//=============================================================================
class Util_Ccf2D
{
public:

	Util_Ccf2D(void);

	~Util_Ccf2D(void);

	void SetSize(int iImageX, int iImageY);

	void SetModes(int iMode1, int iMode2);

	float DoIt(void* pvImage1, void* pvImage2);

private:

	double mCalc(unsigned char* pcImage1, void* pvImage2);

	double mCalc(short* psImage1, void* pvImage2);

	double mCalc(float* pfImage1, void* pvImage2);

	double mCalc(unsigned short* psImage1, void* pvImage2);

	int m_iMode1;

	int m_iMode2;

	int m_iSize[2];
};
