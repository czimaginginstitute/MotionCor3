#pragma once

class Util_TcpSocket
{
public:

	static int CreateSocket(void);

	static void CloseSocket(int iSocket);

	static bool CheckReadEvent(int iSocket, float fSeconds);

	static bool CheckWriteEvent(int iSocket, float fSeconds);

	static int GetPort(int iSocket);
};

