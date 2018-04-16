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

	enum struct ACT{
		ACT_NONE= 0,
		ACT_SLEEP,
		ACT_RESUME,
		ACT_ROBOOT,
		ACT_VERSION,
		ACT_MOD_VERSION,
		ACT_CLOUD_VERSION,
		ACT_MAC,
		ACT_REBIND,
		ACT_SMART_LINK,
		ACT_SMART_AP_LINK,
		ACT_AP_SMART_LINK,
		ACT_FACTORY_TEST,
		ACT_UPLOAD_MAP,
		ACT_CLEAR_MAP,
		ACT_UPLOAD_STATUS,
		ACT_UPLOAD_LAST_CLEANMAP,
		ACT_END,
	};

	S_Wifi();

	~S_Wifi();

	bool init();

	bool deinit();

	int8_t uploadStatus(int msg_code,const uint8_t seq_num);

	bool uploadMap();

	uint8_t setRobotCleanMode(wifi::WorkMode work_mode);

	uint8_t appRemoteCtl(wifi::RemoteControlRxMsg::Cmd data);

	uint8_t clearRealtimeMap(const uint8_t seq_num);

	uint8_t rebind();

	uint8_t syncClock(int year,int mon,int day,int hour,int min,int sec);

	int8_t smartLink();

	uint8_t smartApLink();

	bool factoryTest();

	bool uploadLastCleanData();

	bool setWorkMode(int workmode);

	uint8_t setSchedule(const wifi::SetScheduleRxMsg &sche);

	uint8_t reboot();

	bool resume();

	bool sleep();

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
		return is_active_;
	}

	bool isConnected()
	{

		return is_wifi_connected_;
	}

	bool onRequest()
	{
		return is_Status_Request_;
	}
	uint8_t checkVersion();
	uint8_t checkMAC();

	void taskPushBack(S_Wifi::ACT action);	

	void wifi_send_routine();

	void cacheMapData(const Points map_data);

	void clearMapCache();

	void quit()
	{
		wifi_quit_ = true;
	}

	void resetReceivedWorkMode()
	{
		received_work_mode_ = wifi::WorkMode::MODE_NULL;
	}

	bool receiveShutDown()
	{
		return received_work_mode_ == wifi::WorkMode::SHUTDOWN;
	}

	bool receiveSleep()
	{
		return received_work_mode_ == wifi::WorkMode::SLEEP;
	}

	bool receiveIdle()
	{
		return received_work_mode_ == wifi::WorkMode::IDLE;
	}

	bool receiveRandom()
	{
		return received_work_mode_ == wifi::WorkMode::RANDOM;
	}

	bool receiveFollowWall()
	{
		return received_work_mode_ == wifi::WorkMode::WALL_FOLLOW;
	}

	bool receiveSpot()
	{
		return received_work_mode_ == wifi::WorkMode::SPOT;
	}

	bool receivePlan1()
	{
		return received_work_mode_ == wifi::WorkMode::PLAN1;
	}

	bool receivePlan2()
	{
		return received_work_mode_ == wifi::WorkMode::PLAN2;
	}

	bool receiveHome()
	{
		return received_work_mode_ == wifi::WorkMode::HOMING;
	}

	bool receiveCharge()
	{
		return received_work_mode_ == wifi::WorkMode::CHARGE;
	}

	bool receiveRemote()
	{
		return received_work_mode_ == wifi::WorkMode::REMOTE;
	}

	bool receiveFind()
	{
		return received_work_mode_ == wifi::WorkMode::FIND;
	}

private:

	wifi::RxManager s_wifi_rx_;
	wifi::TxManager s_wifi_tx_;	

	bool is_wifi_connected_;
	bool factory_test_ack_;
	bool isFactoryTest_;
	bool isRegDevice_;
	bool is_Status_Request_;
	bool realtime_map_ack_;
	bool upload_state_ack_;
	bool is_active_;
	bool is_resume_;
	bool is_sleep_;
	bool in_linking_;
	bool wifi_quit_ ;

	wifi::WorkMode robot_work_mode_;
	wifi::WorkMode last_work_mode_;
	wifi::WorkMode received_work_mode_;

	pthread_mutex_t task_lock_;
	pthread_mutex_t map_data_lock_;

	uint64_t MAC_;
	uint32_t moduleVersion_;
	uint32_t cloudVersion_;

	std::deque<ACT> task_list_;

	std::deque<Points> *map_data_buf_;
protected:

	wifi::WorkMode getWorkMode()
	{
		return robot_work_mode_;
	}


};

extern S_Wifi s_wifi;

#endif
