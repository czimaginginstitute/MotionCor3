#include "Util_Folder.h"
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

Util_Folder::Util_Folder(void)
{
}

Util_Folder::~Util_Folder(void)
{
}

char* Util_Folder::GetFolderPath(char* pcFullPath)
{
	if(pcFullPath == NULL) return NULL;
	int iSize = strlen(pcFullPath);
	if(iSize == 0) return NULL;
	int iLastSlash = mFindLastSlash(pcFullPath);
	if(iLastSlash == -1) return NULL;

	char* pcFolderPath = new char[iLastSlash + 1];
	memset(pcFolderPath, 0, sizeof(char) * (iLastSlash + 1));
	memcpy(pcFolderPath, pcFullPath, sizeof(char) * iLastSlash);
	return pcFolderPath;
}

bool Util_Folder::IsValid(char* pcFolderPath)
{
	int iFile = open(pcFolderPath, O_DIRECTORY, S_IRGRP | S_IWUSR);
	if(iFile == -1) return false;
	else return true;
}

void Util_Folder::Create(char* pcFolderPath)
{
	if(pcFolderPath == NULL || strlen(pcFolderPath) == 0) return;
	int iNumParentFolders = mGetNumParentFolders(pcFolderPath);
	for(int i=0; i<iNumParentFolders; i++)
	{	char* pcParentFolder = mGetNthParentFolder(pcFolderPath, i);
		mCreateFolder(pcParentFolder);
		delete[] pcParentFolder;
	}
	mCreateFolder(pcFolderPath);
}

void Util_Folder::mCreateFolder(char* pcFolder)
{
	printf("Create Folder: %s\n", pcFolder);
	if(IsValid(pcFolder)) return;
	int iFile = mkdir(pcFolder, S_IRWXU | S_IRWXG);
}

int Util_Folder::mFindLastSlash(char* pcFolder)
{
	if(pcFolder == NULL || strlen(pcFolder) == 0) return -1;
	char* pc = strrchr(pcFolder, '/');
	if(pc == NULL) return -1;
	int iLocation = pc - pcFolder;
	return iLocation;
}

int Util_Folder::mGetNumParentFolders(char* pcFolderPath)
{
	int iNumParents = 0;
	char* pcParent = strchr(pcFolderPath, '/');
	while(pcParent != NULL)
	{	iNumParents++;
		pcParent = strchr(pcParent + 1, '/');
	}
	return iNumParents;
}

char* Util_Folder::mGetNthParentFolder(char* pcFolderPath, int iNthParent)
{
	char* pcParent = pcFolderPath;
	for(int i=0; i<=iNthParent; i++)
	{	pcParent = strchr(pcParent + 1, '/');
	}
	int iSize = pcParent - pcFolderPath;
	printf("%d  %d  %s\n", iNthParent, iSize, pcParent);
	if(pcParent == NULL) iSize = strlen(pcFolderPath);
	char* pcBuf = new char[iSize + 1];
	memset(pcBuf, 0, sizeof(char) * (iSize + 1));
	memcpy(pcBuf, pcFolderPath, sizeof(char) * iSize);
	return pcBuf;
}
