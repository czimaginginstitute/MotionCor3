#include "Util_String.h"
#include <string.h>
#include <stdlib.h>

void Util_String::RemoveSpace(char* pcString)
{
	if(pcString == 0L || strlen(pcString) == 0) return;
	int iSize = strlen(pcString) + 1;
	char* pcBuf = new char[iSize];
	strcpy(pcBuf, "");

	char* pcToken = NULL;
	for(int i=0; i<iSize; i++)
	{	if(i == 0) 
		{	pcToken = strtok(pcString, " ");
			if(pcToken != NULL) strcpy(pcBuf, pcToken);
		}
		else 
		{	pcToken = strtok(NULL, " ");
			if(pcToken != NULL) strcat(pcBuf, pcToken);
		}
		if(pcToken == NULL) break;
	}
	strcpy(pcString, pcBuf);
	delete[] pcBuf;
}


char* Util_String::GetCopy(char* pcString)
{
	if(pcString == NULL) return NULL;
	int iSize = strlen(pcString) + 1;
	char* pcCopy = new char[iSize];
	strcpy(pcCopy, pcString);
	return pcCopy;
}
