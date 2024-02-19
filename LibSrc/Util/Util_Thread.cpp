#include "Util_Thread.h"
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <stdio.h>

Util_Thread::Util_Thread(void)
{
	m_bCreated = false;
	pthread_mutex_init(&m_aMutex, NULL);
	pthread_cond_init(&m_aCond, NULL);
}

Util_Thread::~Util_Thread()
{
	pthread_mutex_destroy(&m_aMutex);
	pthread_cond_destroy(&m_aCond);
}

void Util_Thread::Start(void)
{
	m_bStop = false;
	int iStatus = pthread_create(&m_aThread, NULL, 
		Util_Thread::ThreadFunc, (void*)this);
	if(iStatus == 0) m_bCreated = true;
	else m_bCreated = false;
}

void Util_Thread::Stop(void)
{
	if(!m_bCreated) return;
	m_bStop = true;
}

bool Util_Thread::IsAlive(void)
{
	if(!m_bCreated) return false;
	//---------------------------
	void* pvRet = 0L;
	int iStatus = pthread_tryjoin_np(m_aThread, &pvRet);
	if(iStatus == EBUSY) return true;
	else if(iStatus == ETIMEDOUT) return true;
	//----------------------------------------
	m_bCreated = false;
	return false;
}

bool Util_Thread::IsCreated(void)
{
	return m_bCreated;
}

bool Util_Thread::WaitForExit(float fSeconds)
{
	if(!m_bCreated) return true;
	//--------------------------
	if(fSeconds < 0) 
	{	pthread_join(m_aThread, NULL);
		m_bCreated = false;
		return true;
	}
	//------------------
	int iSeconds = (int)fSeconds;
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += iSeconds;
	ts.tv_nsec += (int)((fSeconds - iSeconds) * 1e9);
	int iRet = pthread_timedjoin_np(m_aThread, NULL, &ts);
	//----------------------------------------------------
	if(iRet != 0) return false;
	m_bCreated = false;
	return true;
	/*
	else if(iRet == ETIMEDOUT) return false;
	else if(iRet == EBUSY) return false;
	//----------------------------------
	printf("Util_Thread::WaitForExit: iRet = %d\n", iRet);
	m_bCreated = false;
	return true;
	*/
}

bool Util_Thread::WaitSeconds(float fSeconds)
{
	return mWait(fSeconds);
}

void Util_Thread::ThreadMain(void)
{
}

bool Util_Thread::mWait(float fSeconds)
{	
	int iSeconds = (int)fSeconds;
	int iUSeconds = (int)((fSeconds - iSeconds) * 1e6);
	//-------------------------------------------------
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += iSeconds;
	ts.tv_nsec += iUSeconds * 1000;
    	//-----------------------------
	pthread_mutex_lock(&m_aMutex);
	int iStatus = pthread_cond_timedwait(&m_aCond, &m_aMutex, &ts);
	pthread_mutex_unlock(&m_aMutex);
	//------------------------------
	if(iStatus == ETIMEDOUT) return false;
	else return true;
}

void* Util_Thread::ThreadFunc(void* pParam)
{
	Util_Thread* pThread = (Util_Thread*)pParam;
	pThread->ThreadMain();
	return 0L;
}
