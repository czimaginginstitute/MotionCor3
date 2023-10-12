#pragma once

//=============================================================================
//
// This is 2D bilinear interpolation. Callers must make sure each point to
// be interpolated be within the image. A point outside the image will cause
// crash.
//
//=============================================================================
class Util_SIntBase;

class Util_Interpolate
{
public:

	Util_Interpolate(void);

	~Util_Interpolate(void);

	void SetSize(int iSizeX, int iSizeY);

	void SetImage(void* pvImage, int iMode);

	float DoIt(float fX, float fY);

private:

	Util_SIntBase* m_pIntBase;
};

class Util_SIntBase
{
public:

	Util_SIntBase(void);

	virtual ~Util_SIntBase(void);

	void SetSize(int iSizeX, int iSizeY);

	virtual void SetImage(void* pvImage);

	virtual float DoIt(float fX, float fY);

	int m_iSizeX;

	int m_iSizeY;

};

class Util_SIntShort : public Util_SIntBase
{
public:

	Util_SIntShort(void);

	virtual ~Util_SIntShort(void);

	void SetImage(void* pvImage);

	float DoIt(float fX, float fY);

private:

	short* m_psImage;
};

class Util_SIntUShort : public Util_SIntBase
{
public:

	Util_SIntUShort(void);

	virtual ~Util_SIntUShort(void);

	void SetImage(void* pvImage);

	float DoIt(float fX, float fY);

private:

	unsigned short* m_pusImage;
};

class Util_SIntFloat : public Util_SIntBase
{
public:

	Util_SIntFloat(void);

	virtual ~Util_SIntFloat(void);

	void SetImage(void* pvImage);

	float DoIt(float fX, float fY);

private:

	float* m_pfImage;
};
