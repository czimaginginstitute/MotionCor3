#pragma once

class Util_Statistics
{
public:

	static double Mean(char* pcData, int iSize);
	static double Mean(unsigned char* pcData, int iSize);
	static double Mean(short* psData, int iSize);
	static double Mean(unsigned short* psData, int iSize);
	static double Mean(int* piData, int iSize);
	static double Mean(unsigned int* piData, int iSize);
	static double Mean(float* pfData, int iSize);
	static double Mean(double* pdData, int iSize);
	static double Mean(void* pvImage, int iSize, int iMode);

	static double Std(char* pcData, int iSize);
	static double Std(unsigned char* pcData, int iSize);
	static double Std(short* psData, int iSize);
	static double Std(unsigned short* psData, int iSize);
	static double Std(int* piData, int iSize);
	static double Std(unsigned int* piData, int iSize);
	static double Std(float* pfData, int iSize);
	static double Std(double* pdData, int iSize);
	static double Std(void* pvImage, int iSize, int iMode);

	static double Max(char* pcData, int iSize);
	static double Max(unsigned char* pcData, int iSize);
	static double Max(short* psData, int iSize);
	static double Max(unsigned short* psData, int iSize);
	static double Max(int* piData, int iSize);
	static double Max(unsigned int* piData, int iSize);
	static double Max(float* pfData, int iSize);
	static double Max(double* pdData, int iSize);
	static double Max(void* pvImage, int iSize, int iMode);

	static double Min(char* pcData, int iSize);
	static double Min(unsigned char* pcData, int iSize);
	static double Min(short* psData, int iSize);
	static double Min(unsigned short* psData, int iSize);
	static double Min(int* piData, int iSize);
	static double Min(unsigned int* piData, int iSize);
	static double Min(float* pfData, int iSize);
	static double Min(double* pdData, int iSize);
	static double Min(void* pvImage, int iSize, int iMode);
};
