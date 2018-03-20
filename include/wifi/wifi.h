#ifndef __SERIAL_WIFI__
#define __SERIAL_WIFI__
#include "wifi/tx.h"
#include "wifi/rx.h"
#include "wifi/msg.h"
#include "wifi/dev.h"
#include "wifi/packet.h"

class S_Wifi{
	
public:
	S_Wifi();
	~S_Wifi();
	bool init();
	bool deinit();
	uint8_t cloudReplyStatus(const wifi::RxMsg &msg);
	uint8_t testSend();
private:
	wifi::RxManager s_wifi_rx_;
	wifi::TxManager s_wifi_tx_;	

};

extern S_Wifi s_wifi;

#endif
