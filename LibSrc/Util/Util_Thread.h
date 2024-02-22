//=============================================================================
//
// On Linux, you must compile with the -D_REENTRANT option.  This tells
// the C/C++ libraries that the functions must be thread-safe
//
//=============================================================================
#pragma once
#include <pthread.h>

class Util_Thread
{
public:

	Util_Thread(void);
	virtual ~Util_Thread();
	virtual void ThreadMain(void);
	void Start(void);
	void Stop(void);
	bool WaitForExit(float fSeconds);
	bool WaitSeconds(float fSeconds);
	bool IsAlive(void);
	bool IsCreated(void);
private:
	static void* ThreadFunc(void* pvParam);
protected:
	bool mWait(float fSeconds);
	pthread_t m_aThread;
	pthread_cond_t m_aCond;
	pthread_mutex_t m_aMutex;
	bool m_bCreated;
	bool m_bStop;
};

