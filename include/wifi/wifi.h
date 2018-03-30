#ifndef __SERIAL_WIFI__
#define __SERIAL_WIFI__
#include "wifi/tx.h"
#include "wifi/rx.h"
#include "wifi/msg.h"
#include "wifi/dev.h"
#include "wifi/packet.h"
#include "map.h"

class S_Wifi
{
	
public:
	S_Wifi();

	~S_Wifi();

	bool init();

	bool deinit();

	uint8_t replyRobotStatus(int msg_code,const uint8_t seq_num);

	uint8_t replyRealtimePassPath(const Points pass_path);

	uint8_t setRobotCleanMode(wifi::WorkMode work_mode);

	uint8_t appRemoteCtl(wifi::RemoteControlRxMsg::Cmd data);

	uint8_t clearRealtimeMap(const uint8_t seq_num);

	uint8_t rebind();

	uint8_t syncClock(int year,int mon,int day,int hour,int min,int sec);

	uint8_t smartLink();

	uint8_t smartApLink();

	bool factoryTest();

	uint8_t uploadLastCleanData();

	bool setWorkMode(int workmode);

	uint8_t setSchedule(const wifi::SetScheduleRxMsg &sche);

	wifi::WorkMode getWorkMode()
	{
		return robot_work_mode_;
	}

	uint8_t reboot();

	uint8_t resume();

	uint8_t sleep();

	uint32_t getModuleVersion()
	{
		return moduleVersion_;
	}

	uint32_t getCloudVersion()
	{
		return cloudVersion_;
	}

	uint64_t getMAC()
	{
		return MAC_;
	}

	bool isActive()
	{
		return is_wifi_active_;
	}

	bool isConnected()
	{

		return is_wifi_connected_;
	}

	bool onRequest()
	{
		return isStatusRequest_;
	}
	uint8_t checkVersion();
	uint8_t checkMAC();


private:
	wifi::RxManager s_wifi_rx_;
	wifi::TxManager s_wifi_tx_;	

	bool is_wifi_connected_;
	bool inFactoryTest_;
	bool isFactoryTest_;
	bool isRegDevice_;
	bool isStatusRequest_;
	bool is_wifi_active_;

	wifi::WorkMode robot_work_mode_;

	pthread_mutex_t s_wifi_lock_;

	uint64_t MAC_;
	uint32_t moduleVersion_;
	uint32_t cloudVersion_;

};

extern S_Wifi s_wifi;

#endif
