#include "Util_Number.h"
#include "Util_String.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

bool Util_Number::ParseInt(char* pcVal, int& iOutVal)
{
	iOutVal = atoi(pcVal);
	if(iOutVal == 0) return Util_Number::mCheckString(pcVal);
	else return true;
}

bool Util_Number::ParseFloat(char* pcVal, float& fOutVal)
{
	fOutVal = (float)atof(pcVal);
	if(fOutVal == 0.0f) return Util_Number::mCheckString(pcVal);
	else return true;
}

void Util_Number::ToString(double dNumber, char* pcStr)
{	
	if(pcStr == NULL) return;
	sprintf(pcStr, "%f", dNumber);
}

void Util_Number::ToString(int iNumber, char* pcStr)
{	
	if(pcStr == NULL) return;
	sprintf(pcStr, "%d", iNumber);
}

int Util_Number::Round(double dVal)
{
	int iVal = (int)dVal;
	if((iVal - dVal) >= 0.5) return (iVal++);
	else if((dVal - iVal) <= -0.5) return (iVal--);
	else return iVal;
}

bool Util_Number::mCheckString(char* pcVal)
{
	if(pcVal == NULL || strlen(pcVal) == 0) return false;
	return true;
}

bool Util_Number::mIsDigit(char c)
{
	char cVal = c - '0';
	if(cVal >= 0 && cVal <= 9) return true;
	else return false;
}

bool Util_Number::mIsSign(char c)
{
	if(c == '+' || c == '-') return true;
	else return false;
}

bool Util_Number::mIsSpace(char c)
{
	if(c == ' ') return true;
	else return false;
}

