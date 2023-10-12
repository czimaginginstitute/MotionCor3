#pragma once

//=============================================================================
//
// Calculate the mean value of an image. The zero-valued pixels are excluded
// from calculation.
//
//=============================================================================
class Util_ImageMean
{
public:

	Util_ImageMean(void);

	Util_ImageMean(int iSizeX, int iSizeY);

	~Util_ImageMean(void);

	void SetSize(int iSizeX, int iSizeY);

	float Calculate(void* pvImage, int iMode);

private:

	float mCalcUChar(void* pvImage);

	float mCalcShort(void* pvImage);

	float mCalcFloat(void* pvImage);

	float mCalcUShort(void* pvImage);

	int m_iSizeX;

	int m_iSizeY;
};
