#include "Util_SwapByte.h"
#include <stdint.h>
#include <memory.h>

#define swap4(x) ( ((x >> 24) & 0xff) | ((x & 0xff0000) >> 8) \
	| ((x & 0xff00) << 8) | ((x & 0xff) << 24) )

#define swap2(x) (((x & 0xffff) >> 8)|((x & 0xff) << 8))

short Util_SwapByte::DoIt(short sVal)
{
	return swap2(sVal);
}

unsigned short Util_SwapByte::DoIt(unsigned short sVal)
{
	return swap2(sVal);
}

int Util_SwapByte::DoIt(int iVal)
{
	return swap4(iVal);
}

unsigned int Util_SwapByte::DoIt(unsigned int iVal)
{
	return swap4(iVal);
}

float Util_SwapByte::DoIt(float fVal)
{
	unsigned int* piVal = (unsigned int*)(&fVal);
	*piVal = swap4(*piVal);
	return fVal;
}

double Util_SwapByte::DoIt(double dVal)
{
	int64_t* piVal = (int64_t*)(&dVal);
	unsigned int iLow = (unsigned int)((*piVal) & 0xffffffff);
	unsigned int iHigh = (unsigned int)(((*piVal) >> 32) & 0xffffffff);
	iLow = Util_SwapByte::DoIt(iLow);
	iHigh = Util_SwapByte::DoIt(iHigh);
	*piVal = ((int64_t)iLow << 32) + iHigh;
	return dVal;
}
