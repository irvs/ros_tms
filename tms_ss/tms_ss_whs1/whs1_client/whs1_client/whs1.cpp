#include "whs1.h"
#include "include/utwsapi.h"
#include "MyForm.h"

VOID WINAPI rrd1::m_callbackDisconnect(void* _p, void* _h){
	rrd1* _this = reinterpret_cast<rrd1*>(_p);
	MessageBox(NULL, TEXT("ドングルが抜けました"), TEXT("message"), MB_OK);
	_this->m_hRRD1 = 0;
}

char* rrd1::open(){
	m_hRRD1 = UTWSOpenDevice(UTWS_RRD_1, 0);
	UTWSGetErrorMessage(buffer, 256);
	if (m_hRRD1)
		UTWSRRD1SetCallBackDisconnect(m_hRRD1, &rrd1::m_callbackDisconnect, this);
	return buffer;
}

char* rrd1::close(){
	if (m_hRRD1) UTWSRRD1StopReceiving(m_hRRD1);
	is_received = 0;
	UTWSCloseDevice(m_hRRD1);
	m_hRRD1 = 0;
	UTWSGetErrorMessage(buffer, 256);
	return buffer;
}

BOOL rrd1::get_address(){
	return UTWSRRD1GetLocalAddress(m_hRRD1, address);
}

void rrd1::set_address(char* add){
	if (UTWSRRD1SetLocalAddress(m_hRRD1, add)){
		MessageBox(NULL, TEXT("設定しました"), TEXT("message"), MB_OK);
	}
	else{
		UINT err_id = UTWSGetErrorMessage(0, 0);
		if (UTWS_ERR_BAD_DESTINATION_ADDRESS == err_id){
			MessageBox(NULL, TEXT("宛先アドレスが間違っています"), TEXT("message"), MB_OK);
		}
		else{
			MessageBox(NULL, TEXT("失敗しました"), TEXT("message"), MB_OK);
		}
	}
}

void rrd1::ConvertReceivedData(){
	for (int i = 0; i != m_data.data_count; ++i){
		if (now_count + i >= MAX_DATA)now_count -= MAX_DATA;
		hakei[now_count + i] = m_data.data[i].ecg;
		temp[now_count + i] = (int)(m_data.data[i].temp*10);
		if (send_flag == true){
			int temp = (int)(m_data.data[i].temp * 10);
			int senddata = ((m_data.data[i].ecg & 0xffff) << 16) | (temp & 0xffff);
			sock.Send(&senddata,sizeof(senddata));
		}
	}
	now_count += m_data.data_count;
}

VOID WINAPI rrd1::m_callback(void* p, void* h){
	rrd1* _this = reinterpret_cast<rrd1*>(p);
	UINT count = 0;
	while (-1 != (count = UTWSRRD1DataCount(_this->m_hRRD1)) && count){
		UTWSRRD1GetDataEx(_this->m_hRRD1, &_this->m_data);
	}
	_this->ConvertReceivedData();
}

void rrd1::start(){
	if (!m_hRRD1) return;
	UTWSRRD1StopReceiving(m_hRRD1);

	is_received = 1;
	UTWSRRD1StartReceiving(m_hRRD1, 0, &rrd1::m_callback, this);
	return;
}

void rrd1::stop(){
	if (!m_hRRD1) return;
	UTWSRRD1StopReceiving(m_hRRD1);
	is_received = 0;
	return;
}
