#pragma once

class Util_Folder
{
public:

	Util_Folder(void);

	~Util_Folder(void);

	/*
	** Return the folder path given the full path of a file. For
	** example, this method returns "/home/temp" given the full path
	** of "/home/temp/test.mrc".
	*/
	char* GetFolderPath(char* pcFullPath);			/* [out] */

	/*
	** Check if the given folder path is valid.
	*/
	bool IsValid(char* pcFolderPath);

	/*
	** Recursively create folders.
	*/
	void Create(char* pcFolderPath);

private:

	void mCreateFolder(char* pcFolder);

	int mFindLastSlash(char* pcFolder);

	int mGetNumParentFolders(char* pcFolderPath);

	char* mGetNthParentFolder(char* pcFolderPath, int iNthParent);
};
