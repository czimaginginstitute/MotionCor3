#pragma once

class Util_User
{
public:

	static char* GetUserName(bool bSupervisor);		/* [out] */

	static bool IsSupervisor(char* pcUserName);
};
