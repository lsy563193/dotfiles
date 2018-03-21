#ifndef __SERIAL_WIFI__
#define __SERIAL_WIFI__
#include "wifi/tx.h"
#include "wifi/rx.h"
#include "wifi/msg.h"
#include "wifi/dev.h"
#include "wifi/packet.h"

class S_Wifi
{
	
public:
	S_Wifi();

	~S_Wifi();

	bool init();

	bool deinit();

	uint8_t replyRobotStatus(int msg_code,const uint8_t seq_num);

	uint8_t replyRealtimeMap();

	uint8_t setRobotCleanMode(wifi::WorkMode work_mode);

	uint8_t appRemoteCtl(wifi::RemoteControlRxMsg::Cmd data);

	uint8_t clearRealtimeMap(const uint8_t seq_num);

	uint8_t rebind();

	uint8_t syncClock(int year,int mon,int day,int hour,int min,int sec);

	uint8_t smartLink();

	uint8_t smartApLink();

	bool factoryTest();

	uint8_t uploadLastCleanData();

	bool setWorkMode(wifi::WorkMode workmode);

	wifi::WorkMode getWorkMode()
	{
		return robot_work_mode_;
	}

	uint8_t reboot();

	uint8_t resume();

	uint8_t sleep();

	static bool is_wifi_connected_;

	static bool is_cloud_connected_;

	bool isStatusRequest_;

private:
	wifi::RxManager s_wifi_rx_;
	wifi::TxManager s_wifi_tx_;	

	bool inFactoryTest_;
	bool isRegDevice_;

	bool is_wifi_active_;
	wifi::WorkMode robot_work_mode_;

};

extern S_Wifi s_wifi;

#endif
