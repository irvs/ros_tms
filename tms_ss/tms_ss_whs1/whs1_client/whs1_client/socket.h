#ifndef SOCKET_H_
#define SOCKET_H_

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <winsock.h>

#pragma comment(lib,"Wsock32.lib")

class Socket{
private:
	int sock;
	sockaddr_in sinme;
	char *cpData;
	char *IPaddr4;
	int Data;
	unsigned long inet;
public:
	Socket();
	~Socket();
	bool Prepare(int port, const char *IPaddr);
	bool Send(const int *pData, const int DataSize);
	char *IPitoc4(int addr1, int addr2, int addr3, int addr4);
};

extern Socket sock;


#endif