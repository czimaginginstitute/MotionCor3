/********************************************************************
 *
 *
 * This class implements the socket server service. It allows the
 * server program to listen to client request to establish connection
 * and exchange data in either blocking or non-blocking mode.
 * 
 * @note the class implements only single client request, meaning that
 * only one client can connect to the server.
 *
 *********************************************************************/
#pragma once
#include <sys/types.h>

class Util_TcpServer
{
public:

	static bool CreateServer(int iPort = 0);

	static void CloseServer(void);

	static Util_TcpServer* GetInstance(void);

	static void DeleteInstance(void);

	virtual ~Util_TcpServer();

	/* -----------------------------------------
	 * Return  1: Successful connection.
	 * Return  0: No client requests to connect.
	 * Return -1: Failed to connect.
	 * ----------------------------------------- */
	int ConnectClient(float fSeconds);

	void CloseClient(void);

	bool DoSend(void* pvData, size_t iDataSize);

	bool DoRecv(void* pvData, size_t iDataSize);

	bool CanSendNow(float fSeconds = 1.0f);

	bool CanRecvNow(float fSeconds = 1.0f);

private:

	Util_TcpServer();

	static Util_TcpServer* m_pInstance;

	static int m_iServerSock;	// listening socket

	int m_iClientSock;			// client socket
};
