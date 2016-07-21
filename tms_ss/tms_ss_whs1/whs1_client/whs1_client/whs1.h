#ifndef WHS1_H_
#define WHS1_H_

#include "include/utwsapi.h"
#include "socket.h"

#define MAX_DATA 500

class rrd1{
	HANDLE m_hRRD1;
	static VOID WINAPI m_callbackDisconnect(void* _p, void* _h);
	static VOID WINAPI m_callback(void* _p, void* _h);
	char buffer[256];
	RRD1DataEx m_data;
public:
	int is_received;
	char address[11];
	int hakei[MAX_DATA];
	int temp[MAX_DATA];
	int now_count;
	bool send_flag;
	rrd1(){
		is_received = 0;
		now_count = 0;
		send_flag = false;
	}
	char* open();
	char* close();
	BOOL get_address();
	void set_address(char* ad);
	void start();
	void stop();
	void ConvertReceivedData();
};

extern rrd1 rrd;

#endif