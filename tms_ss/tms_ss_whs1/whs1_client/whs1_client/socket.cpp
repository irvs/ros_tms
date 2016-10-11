#include "socket.h"

Socket::Socket(){
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	sock = 0;
}

Socket::~Socket(){
	WSACleanup();
	delete[] IPaddr4;
}

bool Socket::Prepare(int port, const char *IPaddr){
	sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock == INVALID_SOCKET){
		return false;
	}
	sinme.sin_family = AF_INET;
	sinme.sin_port = htons(port);
	inet = inet_addr(IPaddr);
	if (inet != INADDR_NONE){
		sinme.sin_addr.s_addr = inet;
	}
	else{
		return false;
	}
	//connect(sock, (struct sockaddr *)&sinme, sizeof(sinme));
	return true;
}

bool Socket::Send(const int *pData, const int DataSize){
	cpData = (char*)pData;
	if (sendto(sock, cpData, DataSize, 0, (LPSOCKADDR)&sinme, sizeof(sinme)) == SOCKET_ERROR){
		return false;
	}
	return true;
}

char* Socket::IPitoc4(int addr1, int addr2, int addr3, int addr4){
	char *IPaddr4 = new char[11];
	char ipstr1[11], ipstr2[3], ipstr3[3], ipstr4[3];
	sprintf(ipstr1, "%d", addr1);
	sprintf(ipstr2, "%d", addr2);
	sprintf(ipstr3, "%d", addr3);
	sprintf(ipstr4, "%d", addr4);
	strcat(ipstr1, ".");
	strcat(ipstr1, ipstr2);
	strcat(ipstr1, ".");
	strcat(ipstr1, ipstr3);
	strcat(ipstr1, ".");
	strcat(ipstr1, ipstr4);
	strcat(ipstr1, "\0");

	strcpy(IPaddr4, ipstr1);

	return IPaddr4;
}