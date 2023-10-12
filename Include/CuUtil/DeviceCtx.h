#pragma once
#include <cuda.h>

class CDeviceCtx
{
public:

	static int GetDeviceCount(void);
	
	CDeviceCtx(void);

	~CDeviceCtx(void);

	void SetDevice(int iDevice = 0);

private:

	int m_iDevice;
};
