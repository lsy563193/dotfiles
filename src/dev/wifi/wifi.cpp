#include "wifi/wifi.h"
#include <ros/ros.h>
#include <time.h>
#include "speaker.h"
#include "mode.hpp"
#include "move_type.hpp"
#include "serial.h"
#include "error.h"
#include "battery.h"
#include "event_manager.h"
#include "dev.h"

#define CLOUD_DOMAIN_ID 5479
#define CLOUD_SUBDOMAIN_ID 6369
#define CLOUD_AP "Robot"
#define CLOUD_KEY "BEEE3F8925CC677AC1F3D1D9FEBC8B8632316C4B98431632E11933E1FB3C2167E223EFCC653ED3324E3EA219CCCD3E97D8242465C9327A91578901CA65015DB1C80DD4A0F45C5CC7DF1267A2FD5C00E7BD3175C2BB08BA8CFA886CCED2F70D214FB2CC88ECCA6BB1B5F4E6EE9948D424"

S_Wifi s_wifi;

S_Wifi::S_Wifi():is_wifi_connected_(false)
					,is_Status_Request_(false)
					,factory_test_ack_(false)
					,isFactoryTest_(false)
					,isRegDevice_(false)
					,is_resume_(false)
					,is_active_(false)
					,is_sleep_(false)
					,in_linking_(false)
					,task_lock_(PTHREAD_MUTEX_INITIALIZER)
					,map_data_lock_(PTHREAD_MUTEX_INITIALIZER)
					,wifi_quit_(false)
					,realtime_map_ack_(false)
					,upload_state_ack_(false)
{
	init();
	map_data_buf_ = new std::deque<Points>();
}
S_Wifi::~S_Wifi()
{
	deinit();
}

bool S_Wifi::deinit()
{
	taskPushBack(ACT::ACT_SLEEP);
	quit();
	delete map_data_buf_;
	return true;
}

bool S_Wifi::init()
{
	robot_work_mode_ = wifi::WorkMode::IDLE;
	INFO_BLUE(" wifi register msg... ");
	//-----on reg device ,connect or disconnect-----
	//regestory request
	s_wifi_rx_.regOnNewMsgListener<wifi::RegDeviceRequestRxMsg>(
		[&]( const wifi::RxMsg &a_msg ) 
		{
			is_active_ = true;
			wifi::RegDeviceTxMsg p(	wifi::RegDeviceTxMsg::CloudEnv::MAINLAND_DEV,
									{0, 0, 1},
									CLOUD_DOMAIN_ID,
									CLOUD_SUBDOMAIN_ID,
									wifi::RegDeviceTxMsg::toKeyArray(CLOUD_KEY),
									a_msg.seq_num());
			s_wifi_tx_.push(std::move( p )).commit();
			isRegDevice_ = true;
			wifi_led.setMode(LED_FLASH,WifiLed::state::on);
			if(isFactoryTest_)
				speaker.play( VOICE_WIFI_CONNECTED,false);
		});
/*
	s_wifi_rx_.regOnNewMsgListener<wifi::wifiConnectedNotifRxMsg>(
			[this]( const wifi::RxMsg &a_msg ) {
				is_wifi_connected_ = true;
				wifi_led.isMaxInClean(LED_FLASH, WifiLed::state::on);
				speaker.play( VOICE_WIFI_CONNECTED,false);
			});
*/
	//wifi disconncet
	s_wifi_rx_.regOnNewMsgListener<wifi::wifiDisconnectedNotifRxMsg>(
			[&]( const wifi::RxMsg &a_msg ) {
				is_wifi_connected_ = false;
				wifi_led.setMode(LED_STEADY, WifiLed::state::off);
				//speaker.play( VOICE_WIFI_UNCONNECTED_UNOFFICIAL,false);
			});

	//cloud connect
	s_wifi_rx_.regOnNewMsgListener<wifi::CloudConnectedNotifRxMsg>(
			[&]( const wifi::RxMsg &a_msg ) {
				is_wifi_connected_ = true;
				wifi_led.setMode(LED_STEADY,WifiLed::state::on);
				if(isRegDevice_ && in_linking_){
					//speaker.play( VOICE_CLOUD_CONNECTED,false);
					speaker.play( VOICE_WIFI_CONNECTED,false);
					isRegDevice_ = false;
					in_linking_ = false;
				}
			});

	//cound disconnect
	s_wifi_rx_.regOnNewMsgListener<wifi::CloudDisconnectedNotifRxMsg>(
				[this](const wifi::RxMsg &a_msg){
				is_wifi_connected_ = false;
				wifi_led.setMode(LED_STEADY,WifiLed::state::off);
				//speaker.play(VOICE_CLOUD_UNCONNECTED_UNOFFICIAL,false);
				});

	//-----app query -----
	//query device status
	s_wifi_rx_.regOnNewMsgListener<wifi::QueryDeviceStatusRxMsg>(
			[&]( const wifi::RxMsg &a_msg ) {
				const wifi::QueryDeviceStatusRxMsg &msg = static_cast<const wifi::QueryDeviceStatusRxMsg&>( a_msg );
				uploadStatus( msg.MSG_CODE,msg.seq_num());
				is_wifi_connected_ = true;
				wifi_led.setMode(LED_STEADY, WifiLed::state::on);
			});
	//query schedule
	s_wifi_rx_.regOnNewMsgListener<wifi::QueryScheduleStatusRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::QueryScheduleStatusRxMsg &msg = static_cast<const wifi::QueryScheduleStatusRxMsg&>( a_msg );
				//get appointment 
				std::vector<wifi::ScheduleStatusTxMsg::Schedule> vec_sch;
				std::vector<Appointment::st_appmt> appmts = appmt_obj.get();
				for(int i = 1;i<appmts.size();i++)
				{
					wifi::ScheduleStatusTxMsg::Schedule sche(i,
								appmts[i].enable,
								appmts[i].week,
								appmts[i].hour,
								appmts[i].mint);
					vec_sch.push_back(sche);
				}
				//ack to cloud
				wifi::ScheduleStatusTxMsg p(
							vec_sch,
							msg.seq_num()
							);
				s_wifi_tx_.push(std::move(p)).commit();
				INFO_BLUE("receive query schedule");	
			});
	//query consumption
	s_wifi_rx_.regOnNewMsgListener<wifi::QueryConsumableStatusRxMsg>(
			[&]( const wifi::RxMsg &a_msg ) {
				const wifi::QueryConsumableStatusRxMsg &msg = static_cast<const wifi::QueryConsumableStatusRxMsg&>( a_msg );
				uint16_t worktime = (uint16_t)(robot_timer.getWorkTime()/3600);
				//ack
				wifi::ConsumableStatusTxMsg p(
							worktime,
							0x64,0x64,0x64,0x64,0x64,//todo
							msg.seq_num()	
						);
				s_wifi_tx_.push(std::move(p)).commit();

			});

	//-----app control -----
	//set clean mode
	s_wifi_rx_.regOnNewMsgListener<wifi::SetModeRxMsg>(
			[&]( const wifi::RxMsg &a_msg){
				const wifi::SetModeRxMsg &msg = static_cast<const wifi::SetModeRxMsg&>( a_msg );
				//ack
				wifi::Packet p(
							-1,
							msg.seq_num(),
							0,
							msg.msg_code(),
							msg.data()
							);
				s_wifi_tx_.push(std::move(p)).commit();
				//set mode
				setRobotCleanMode(msg.getWorkMode());
				robot_work_mode_ = msg.getWorkMode();
				taskPushBack(ACT::ACT_UPLOAD_STATUS);
			});
	//set room mode
	s_wifi_rx_.regOnNewMsgListener<wifi::SetRoomModeRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::SetRoomModeRxMsg &msg = static_cast<const wifi::SetRoomModeRxMsg &>( a_msg );
				//ack
				wifi::Packet p(
							-1,
							msg.seq_num(),
							0,
							msg.msg_code(),
							msg.data()
							);
				s_wifi_tx_.push(std::move(p)).commit();
			});
	//set max clean power
	s_wifi_rx_.regOnNewMsgListener<wifi::SetMaxCleanPowerRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::SetMaxCleanPowerRxMsg &msg = static_cast<const wifi::SetMaxCleanPowerRxMsg&>( a_msg );
				// todo : this setting has something wrong.
				// Setting for pump and swing motor.
				if(msg.isMop())
				{
					water_tank.setPumpMode(WaterTank::PUMP_HIGH);
					water_tank.setTankMode(WaterTank::TANK_HIGH);
				}
				else
				{
					water_tank.setPumpMode(WaterTank::PUMP_MID);
					water_tank.setTankMode(WaterTank::TANK_LOW);
				}
				// Setting for vacuum.
				vacuum.isMaxInClean(msg.isVacuum());
				if (vacuum.isOn())
					vacuum.setCleanState();

				/*if (!water_tank.checkEquipment(true))
					if(msg.isMop())
						water_tank.setPumpMode(WaterTank::PUMP_HIGH);
					else
						water_tank.setPumpMode(WaterTank::PUMP_MID);
				else
					vacuum.isMaxInClean(msg.isVacuum());*/
				//ack
				wifi::MaxCleanPowerTxMsg p(vacuum.isMaxInClean(),water_tank.getMode() == WaterTank::PUMP_HIGH);
				s_wifi_tx_.push( std::move(p)).commit();
			});
	//remote control
	s_wifi_rx_.regOnNewMsgListener<wifi::RemoteControlRxMsg>(
			[&]( const wifi::RxMsg &a_msg ) {
				const wifi::RemoteControlRxMsg &msg = static_cast<const wifi::RemoteControlRxMsg&>( a_msg );
				appRemoteCtl(msg.getCmd());
				//ack
				wifi::Packet p(
							-1,
							msg.seq_num(),
							0,
							msg.msg_code(),
							msg.data()
							);
				s_wifi_tx_.push(std::move(p)).commit();
			});
	//set schedule
	s_wifi_rx_.regOnNewMsgListener<wifi::SetScheduleRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::SetScheduleRxMsg &msg = static_cast<const wifi::SetScheduleRxMsg&>(a_msg);

				this->setSchedule(msg);
				//ack;
				wifi::Packet p(
							-1,
							msg.seq_num(),
							0,
							msg.msg_code(),
							msg.data()
							);
				s_wifi_tx_.push(std::move(p)).commit();
			});
	//reset consumable status
	s_wifi_rx_.regOnNewMsgListener<wifi::ResetConsumableStatusRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::ResetConsumableStatusRxMsg &msg = static_cast<const wifi::ResetConsumableStatusRxMsg&>( a_msg );
				//ack
				wifi::Packet p(
							-1,
							msg.seq_num(),
							0,
							msg.msg_code(),
							msg.data()
							);
				s_wifi_tx_.push(std::move(p)).commit();
				//todo
			});

	//sync clock
	s_wifi_rx_.regOnNewMsgListener<wifi::SyncClockRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::SyncClockRxMsg &msg = static_cast<const wifi::SyncClockRxMsg&>( a_msg );
				Timer::DateTime date_time;
				date_time.year = msg.getYear();
				date_time.month = msg.getMonth();
				date_time.day = msg.getDay();
				date_time.hour = msg.getHour();
				date_time.mint = msg.getMin();
				date_time.sec = msg.getSec();
				robot_timer.setRealTime(date_time);
				//ack
				wifi::Packet p(
							-1,
							msg.seq_num(),
							0,
							msg.msg_code(),
							msg.data()
							);
				s_wifi_tx_.push(std::move(p)).commit();
			});
	//set status requset
	s_wifi_rx_.regOnNewMsgListener<wifi::RealtimeStatusRequestRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::RealtimeStatusRequestRxMsg &msg = static_cast<const wifi::RealtimeStatusRequestRxMsg&>( a_msg );
				is_Status_Request_ = msg.isEnable()?true:false;
				//ack
				wifi::Packet p(
						-1,
						msg.seq_num(),
						0,
						msg.msg_code(),
						msg.data()
						);
				s_wifi_tx_.push(std::move(p)).commit();
			});
	//set do not disturb
	s_wifi_rx_.regOnNewMsgListener<wifi::SetDoNotDisturbRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::SetDoNotDisturbRxMsg &msg = static_cast<const wifi::SetDoNotDisturbRxMsg&>( a_msg );
				//ack
				wifi::Packet p(
						-1,
						msg.seq_num(),
						0,
						msg.msg_code(),
						msg.data()
						);
				s_wifi_tx_.push(std::move(p)).commit();
				//todo
			}
	);
	//factory reset
	s_wifi_rx_.regOnNewMsgListener<wifi::FactoryResetRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::FactoryResetRxMsg &msg = static_cast<const wifi::FactoryResetRxMsg&>( a_msg );
				//ack
				wifi::Packet p(
						-1,
						msg.seq_num(),
						0,
						msg.msg_code(),
						msg.data()
						);
				s_wifi_tx_.push(std::move(p)).commit();
				//todo
			}
	);
	
	//-----ack------
	//factory test ack
	s_wifi_rx_.regOnNewMsgListener<wifi::FactoryTestRxMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::FactoryTestRxMsg &msg = static_cast<const wifi::FactoryTestRxMsg&>( a_msg );
				factory_test_ack_= true;
				wifi_led.setMode(LED_FLASH,WifiLed::state::on);
			}
	);
	//upload status ack
	s_wifi_rx_.regOnNewMsgListener<wifi::DeviceStatusUploadAckMsg>(
					[&](const wifi::RxMsg & a_msg){
				const wifi::DeviceStatusUploadAckMsg &msg = static_cast<const wifi::DeviceStatusUploadAckMsg&>(a_msg);
				upload_state_ack_ = true;
		});

	//realtime map upload ack
	s_wifi_rx_.regOnNewMsgListener<wifi::RealtimeMapUploadAckMsg>(
			[&](const wifi::RxMsg &a_msg){
				realtime_map_ack_ = true;
			});
	//claer realtime map ack
	s_wifi_rx_.regOnNewMsgListener<wifi::ClearRealtimeMapAckMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::ClearRealtimeMapAckMsg &msg = static_cast<const wifi::ClearRealtimeMapAckMsg&>( a_msg );
			});
	//resume ack
	s_wifi_rx_.regOnNewMsgListener<wifi::wifiResumeAckMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::wifiResumeAckMsg &msg = static_cast<const wifi::wifiResumeAckMsg&>(a_msg);	
				INFO_BLUE("RESUME ACK");
				is_resume_= true;
				is_active_ = true;
				is_sleep_ = false;
				if(!isFactoryTest_)
					checkVersion();
				});
	//suspend ack
	s_wifi_rx_.regOnNewMsgListener<wifi::wifiSuspendAckMsg>(
			[&](const wifi::RxMsg &a_msg){
				const wifi::wifiSuspendAckMsg &msg = static_cast<const wifi::wifiSuspendAckMsg&>(a_msg);
					is_sleep_ = true;
					is_resume_= false;
					wifi_led.setMode(LED_STEADY, WifiLed::state::off);
				});
	// version ack
	s_wifi_rx_.regOnNewMsgListener<wifi::wifiVersionAckMsg>(
					[&](const wifi::RxMsg & a_msg){
				const wifi::wifiVersionAckMsg &msg = static_cast<const wifi::wifiVersionAckMsg&>(a_msg);
				moduleVersion_ = msg.getModuleVersion();
				cloudVersion_ = msg.getCloudVersion();
				ROS_INFO("version %d,cloud %d"
							,moduleVersion_,
							cloudVersion_);
				checkMAC();
		});

	// MAC ack
	s_wifi_rx_.regOnNewMsgListener<wifi::wifiMACAckMsg>(
					[&](const wifi::RxMsg & a_msg){
				const wifi::wifiMACAckMsg &msg = static_cast<const wifi::wifiMACAckMsg&>(a_msg);
				MAC_ = msg.getMAC();
		});

	INFO_BLUE("register done ");
	return true;
}

int8_t S_Wifi::uploadStatus(int msg_code,const uint8_t seq_num)
{
	if(!is_wifi_connected_ )
		return -1;
	uint8_t error_code = 0;
	wifi::DeviceStatusBaseTxMsg::CleanMode box;
	box = water_tank.getEquimentStatus()? wifi::DeviceStatusBaseTxMsg::CleanMode::WATER_TANK: wifi::DeviceStatusBaseTxMsg::CleanMode::DUST;

//	while(ros::ok() && robot::instance()->p_mode == nullptr);
//	setWorkMode((int)robot::instance()->p_mode->getNextMode());

	switch (error.get())
	{
		case ERROR_CODE_NONE:
			error_code = 0x00;
			break;

		case ERROR_CODE_BUMPER:
			error_code = 0x01;
			break;

		case ERROR_CODE_OBS:
			error_code = 0x11;
			break;

		case ERROR_CODE_LEFTWHEEL:
			error_code = 0x51;
			break;

		case ERROR_CODE_RIGHTWHEEL:
			error_code = 0x52;
			break;

		case ERROR_CODE_LEFTBRUSH:
			error_code = 0x41;
			break;

		case ERROR_CODE_RIGHTBRUSH:
			error_code = 0x42;
			break;

		case ERROR_CODE_CLIFF:
			error_code = 0x21;
			break;

		case ERROR_CODE_STUCK:
			error_code = 0xd1;
			break;

		case ERROR_CODE_MAINBRUSH:
			error_code = 0x61;
			break;

		case ERROR_CODE_VACUUM:
			error_code = 0x71;
			break;

		case ERROR_CODE_WATERTANK:
			error_code = 0x81;
			break;

		case ERROR_CODE_BTA:
			error_code = 0xa1;
			break;

		case ERROR_CODE_DUSTBIN:
			error_code = 0x91;
			break;

		case ERROR_CODE_GYRO:
			error_code = 0xB1;
			break;

		case ERROR_CODE_LIDAR:
			error_code = 0xc1;
			break;

		case ERROR_CODE_AIRPUMP:
			error_code = 0x82;
			break;

		case ERROR_CODE_FILTER:
			error_code = 0x93;
			break;

		case ERROR_CODE_OTHER:
			error_code = 0xe1;
			break;

		default:
			error_code = 0x00;
	}

	if(msg_code == 0xc8)// auto upload
	{
		int upload_state_ack_cnt=0;
		do{
			if(upload_state_ack_cnt++ > 10)
				return -1;
			wifi::DeviceStatusUploadTxMsg p(
					robot_work_mode_,
					wifi::DeviceStatusBaseTxMsg::RoomMode::LARGE,//default set large
					box,
					serial.getSendData(CTL_VACCUM_PWR),
					serial.getSendData(CTL_BRUSH_MAIN),
					battery.getPercent(),
					0x01,//notify sound wav
					0x01,//led on/off
					error_code,
					seq_num);

			s_wifi_tx_.push(std::move( p )).commit();
			usleep(400000);
		}while(ros::ok() && !upload_state_ack_);
		upload_state_ack_ = false;
	}
	else if(msg_code == 0x41)//app check upload
	{
		wifi::DeviceStatusReplyTxMsg p(
				robot_work_mode_,
				wifi::DeviceStatusBaseTxMsg::RoomMode::LARGE,//default set larger
				box,
				serial.getSendData(CTL_VACCUM_PWR),
				serial.getSendData(CTL_BRUSH_MAIN),
				battery.getPercent(),
				0x01,//notify sound wav
				0x01,//led on/off
				error_code,
				seq_num);
		s_wifi_tx_.push(std::move( p )).commit();
	}
	return 0;
}

bool S_Wifi::uploadMap()
{
	if(!is_wifi_connected_ )
		return false;
	uint32_t time  = (uint32_t)ros::Time::now().toSec();
	std::vector<uint8_t> map_data;
	std::vector<std::vector<uint8_t>> map_pack;
	int pack_cnt=0;
	int byte_cnt=0;

	if(	robot::instance()->p_mode == nullptr ||
				robot::instance()->p_mode->getNextMode() != Mode::cm_navigation)
		return false;

	if(map_data_buf_->size()  == 0)
		return false;

	pthread_mutex_lock(&map_data_lock_);
	Points pass_path = map_data_buf_->front();
	pthread_mutex_unlock(&map_data_lock_);

	if(!pass_path.empty())
	{

		GridMap g_map;
		if(robot::instance()->p_mode != nullptr)
		{
			auto mode = boost::dynamic_pointer_cast<ACleanMode>(robot::instance()->p_mode);
			g_map = mode->clean_map_;
		}
		else
			return false;
		uint16_t clean_area = (uint16_t)(g_map.getCleanedArea()*CELL_SIZE*CELL_SIZE*100);
		//push clean area and work time
		map_data.push_back((uint8_t)((clean_area&0xff00)>>8));
		map_data.push_back((uint8_t)clean_area);
		map_data.push_back((uint8_t)((robot_timer.getWorkTime()&0x0000ff00)>>8));
		map_data.push_back((uint8_t)robot_timer.getWorkTime());
		byte_cnt+=4;
		//--pack date
		for(auto &&p_it:pass_path)
		{
			int16_t i = p_it.toCell().x;
			int16_t j = p_it.toCell().y;
			if(pack_cnt>=255)
			{
				pack_cnt=255;
				ROS_ERROR("%s,%d,MAP TOO BIG TO SEND",__FUNCTION__,__LINE__);
				break;
			}
			for(int16_t pos_x = i-1;pos_x<=i+1;pos_x++)
			{
				for(int16_t pos_y = j-1;pos_y<=j+1;pos_y++)
				{
					CellState c_state = g_map.getCell(CLEAN_MAP,pos_x,pos_y);
					if(c_state == CLEANED)
					{
						map_data.push_back((uint8_t) (pos_x>>8));
						map_data.push_back((uint8_t) (0x00ff&pos_x));
						map_data.push_back((uint8_t) (pos_y>>8));
						map_data.push_back((uint8_t) (0x00ff&pos_y));
						byte_cnt+=4;
					}
					if(byte_cnt>= 480)
					{
						map_pack.push_back(map_data);
						map_data.clear();
						pack_cnt+=1;
						//push clean area and work time
						map_data.push_back((uint8_t)((clean_area&0xff00)>>8));
						map_data.push_back((uint8_t)clean_area);
						map_data.push_back((uint8_t)((robot_timer.getWorkTime()&0x0000ff00)>>8));
						map_data.push_back((uint8_t)robot_timer.getWorkTime());
						byte_cnt=4;
					}

				}
			}

		}
		if(byte_cnt >4 && byte_cnt < 480)
				map_pack.push_back(map_data);
		ROS_INFO("%s,%d,map_pack size %ld",__FUNCTION__,__LINE__,map_pack.size());
		//upload map and wait ack
		int timeout_cnt = 0;
		for(int k=1;k<=map_pack.size();k++)
		{
			do{
				if(timeout_cnt++ > 10)
					return false;
				wifi::RealtimeMapUploadTxMsg p(
									time,
									(uint8_t)k,
									(uint8_t)map_pack.size(),
									map_pack[k-1]
									);
				s_wifi_tx_.push(std::move(p)).commit();
				usleep(350000);
			}while(ros::ok() && !realtime_map_ack_);
			timeout_cnt = 0;
		}
		realtime_map_ack_ =false;
		pthread_mutex_lock(&map_data_lock_);
		map_data_buf_->pop_front();
		pthread_mutex_unlock(&map_data_lock_);

	}
	return true;
}

uint8_t S_Wifi::setRobotCleanMode(wifi::WorkMode work_mode)
{
	ROS_INFO("%s,%d,work mode  = %d",__FUNCTION__,__LINE__,(int)work_mode);
	switch(work_mode)
	{
		case wifi::WorkMode::SLEEP:
//			ev.key_long_pressed = true;//set sleep mode
			received_work_mode_ = work_mode;
			break;
		case wifi::WorkMode::IDLE:
			if(last_work_mode_ == wifi::WorkMode::PLAN1
						|| last_work_mode_ == wifi::WorkMode::WALL_FOLLOW
						|| last_work_mode_ == wifi::WorkMode::SPOT
						|| last_work_mode_ == wifi::WorkMode::HOMING
						|| last_work_mode_ == wifi::WorkMode::FIND
						|| last_work_mode_ == wifi::WorkMode::RANDOM
						|| last_work_mode_ == wifi::WorkMode::REMOTE )//get last mode
			{
//				remote.set(REMOTE_CLEAN);
				received_work_mode_ = work_mode;
//				beeper.beepForCommand(VALID);
				//-- tmp debug

				if(last_work_mode_ == wifi::WorkMode::PLAN1)
					taskPushBack(ACT::ACT_UPLOAD_LAST_CLEANMAP);
			}
			else{
				ROS_INFO("%s %d: Invalid idle cmd.", __FUNCTION__, __LINE__);
//				beeper.beepForCommand(INVALID);
			}
			INFO_BLUE("receive mode idle");
			break;
		case wifi::WorkMode::RANDOM:
//			beeper.beepForCommand(INVALID);
//			remote.set(REMOTE_CLEAN);
			received_work_mode_ = work_mode;
			INFO_BLUE("receive mode random");
			break;
		case wifi::WorkMode::WALL_FOLLOW:
			received_work_mode_ = work_mode;
//			remote.set(REMOTE_WALL_FOLLOW);
//			beeper.beepForCommand(VALID);
			INFO_BLUE("receive mode wall follow");
			break;
		case wifi::WorkMode::SPOT:
			received_work_mode_ = work_mode;
//			remote.set(REMOTE_SPOT);
//			beeper.beepForCommand(VALID);
			INFO_BLUE("receive mode spot");
			break;
		case wifi::WorkMode::PLAN1://plan 1
//			beeper.beepForCommand(VALID);
			received_work_mode_ = work_mode;
//			remote.set(REMOTE_CLEAN);//clean key
			INFO_BLUE("receive mode plan1");
			break;
		case wifi::WorkMode::PLAN2://plan 2
//			beeper.beepForCommand(VALID);
			received_work_mode_ = work_mode;
//			remote.set(REMOTE_CLEAN);//clean key
			INFO_BLUE("receive mode plan2");
			break;
		case wifi::WorkMode::HOMING:
			/*if(last_work_mode_ == wifi::WorkMode::HOMING)
			{
				remote.set(REMOTE_CLEAN);
				beeper.beepForCommand(INVALID);
			}
			else
			{
				remote.set(REMOTE_HOME);//go home
				beeper.beepForCommand(VALID);
			}*/
			received_work_mode_ = work_mode;
//			beeper.beepForCommand(VALID);
			INFO_BLUE("receive mode gohome");
			break;
		case wifi::WorkMode::CHARGE:
			INFO_BLUE("remote charger command ");
//			beeper.beepForCommand(INVALID);
			break;
		case wifi::WorkMode::REMOTE:
			received_work_mode_ = work_mode;
			remote.set(REMOTE_FORWARD);//remote hand mode
			INFO_BLUE("remote hand mode command ");
			break;

		case wifi::WorkMode::FIND:
//			beeper.beepForCommand(VALID);
#if DEBUG_ENABLE
			speaker.play(VOICE_IM_HERE_UNOFFICIAL,false);
#endif
			INFO_BLUE("remote app find home mode command ");
			break;

	}
	last_work_mode_ = work_mode;
	return 0;
}

uint8_t S_Wifi::clearRealtimeMap(const uint8_t seq_num)
{
	if(!is_wifi_connected_ )
		return 1;
	wifi::ClearRealtimeMapTxMsg p(true,seq_num);
	s_wifi_tx_.push(std::move( p )).commit();
	return 0;
}

uint8_t S_Wifi::appRemoteCtl(wifi::RemoteControlRxMsg::Cmd data)
{
	if(!is_wifi_connected_ )
		return 1;
	robot_work_mode_ = wifi::WorkMode::REMOTE;
	last_work_mode_ = robot_work_mode_;
	ROS_INFO("%s %d: Receive remote command(%d).", __FUNCTION__, __LINE__, data);
	switch(data)
	{
		case wifi::RemoteControlRxMsg::Cmd::FORWARD:
			remote.set(REMOTE_FORWARD);//forward control
			break;
		case wifi::RemoteControlRxMsg::Cmd::BACKWARD://backward control
			beeper.beepForCommand(INVALID);
			remote.reset(); //tmp set stop
			break;
		case wifi::RemoteControlRxMsg::Cmd::LEFT:
			remote.set(REMOTE_LEFT);//left control
			break;
		case wifi::RemoteControlRxMsg::Cmd::RIGHT:
			remote.set(REMOTE_RIGHT);//right control
			break;
		case wifi::RemoteControlRxMsg::Cmd::STOP:
			remote.set(remote.get());
			remote.reset();//stop control
			break;
	}
	return 0;
}

uint8_t S_Wifi::syncClock(int year,int mon,int day,int hour,int minu,int sec)
{
	char date_time[50];
	sprintf(date_time,"date -s \"%02d-%02d-%02d %02d:%02d:%02d\""
				,year,mon,day,hour,minu,sec);
	system(date_time);
	
	robot_timer.initWorkTimer();
//	IAction::updateStartTime();

	struct tm *local_time;
	time_t ltime;
	time(&ltime);
	local_time = localtime(&ltime);
	ROS_INFO("%s,%d,local time %s",__FUNCTION__,__LINE__,asctime(local_time));
	return 0;
}

uint8_t S_Wifi::rebind()
{
	//if(!is_active_)
	//{
	//	INFO_RED("WIFI NOT ACTIVE");
	//	return -1;
	//}
	INFO_BLUE("wifi rebind");
	wifi::ForceUnbindTxMsg p(0x00);//no responed
	s_wifi_tx_.push(std::move(p)).commit();
	is_wifi_connected_ = false;
#if DEBUG_ENABLE
	//speaker.play(VOICE_WIFI_UNBIND,false);
#endif
	return 0;
}

int8_t S_Wifi::smartLink()
{
	//if(!is_active_)
	//{
	//	INFO_RED("WIFI NO ACTIVE");
	//	return -1;
	//}
	INFO_BLUE("SMART LINK");
	wifi::SmartLinkTxMsg p(0x00);//no responed
	s_wifi_tx_.push( std::move(p)).commit();
#if DEBUG_ENABLE
	//speaker.play(VOICE_WIFI_SMART_LINK_UNOFFICIAL,false);
	speaker.play(VOICE_WIFI_CONNECTING,false);
#endif
	wifi_led.setMode(LED_FLASH,WifiLed::state::on);
	in_linking_ = true;
	return 0;
}

uint8_t S_Wifi::smartApLink()
{
	INFO_BLUE("AP LINK");
	wifi::SmartApLinkTxMsg p(CLOUD_AP,0x00);
	s_wifi_tx_.push(std::move(p)).commit();
	speaker.play(VOICE_WIFI_CONNECTING,false);
	wifi_led.setMode(LED_FLASH,WifiLed::state::on);
	return 0;
}

bool S_Wifi::uploadLastCleanData()
{
	if(!is_wifi_connected_ )
		return false;
	INFO_BLUE("UPLOAD LAST STATE & MAP");
	uint32_t time = ros::Time::now().toSec();
	std::vector<uint8_t> map_data;
	std::vector<std::vector<uint8_t>> map_pack;
	map_pack.clear();
	if( getWorkMode() == wifi::WorkMode::PLAN1)
	{
		GridMap g_map;
		if(robot::instance()->p_mode != nullptr)
		{
			auto mode = boost::dynamic_pointer_cast<ACleanMode>(robot::instance()->p_mode);
			g_map = mode->clean_map_;
		}
		else
			return false;

		uint16_t clean_area = (uint16_t)(g_map.getCleanedArea()*CELL_SIZE*CELL_SIZE*100);
		int16_t x_min,x_max,y_min,y_max;
		g_map.getMapRange(CLEAN_MAP,&x_min,&x_max,&y_min,&y_max);
		int16_t col_n = (int16_t)ceilf((y_max-y_min+1)/8.0);
		int16_t row_n = (int16_t)(480/col_n);
		int16_t pack_n = (int16_t)ceilf((x_max-x_min+1)/(row_n*1.0));
		for(int p = 0;p<pack_n;p++)
		{
			//push clean area and work time
			map_data.push_back((uint8_t)(time>>24));
			map_data.push_back((uint8_t)(time>>16));
			map_data.push_back((uint8_t)(time>>8));
			map_data.push_back((uint8_t)(time));
			map_data.push_back((uint8_t)(robot_timer.getWorkTime()>>8));
			map_data.push_back((uint8_t)robot_timer.getWorkTime());
			map_data.push_back((uint8_t)(clean_area>>8));
			map_data.push_back((uint8_t)clean_area);
			map_data.push_back((uint8_t)p);
			map_data.push_back((uint8_t)pack_n);
			map_data.push_back((uint8_t)col_n);
			uint8_t tmp_byte  = 0;
			for(int r = 0;r<row_n;r++)
			{
				for(int col = 0;col<col_n;col++)
				{
					tmp_byte = 0;
					for(int bi= 0;bi<8;bi++)
					{ 
						CellState c_state = g_map.getCell(CLEAN_MAP, p*row_n+r+x_min, col*8+bi+y_min);
						if(c_state == CLEANED)
							tmp_byte |= 0x80>>bi;
						else
							tmp_byte &= ~(0x80>>bi);
					}
					map_data.push_back(tmp_byte);
				}

			}
			map_pack.push_back(map_data);
			map_data.clear();
		}
		for(int i = 0;i<map_pack.size();i++)
		{
			wifi::Packet p(-1,0x01,0,0xc9,map_pack[i]);
			s_wifi_tx_.push(std::move(p)).commit();
		}
		ROS_INFO("%s,%d,\033[1;42;31mmap_pack size %ld\033[0m",__FUNCTION__,__LINE__,map_pack.size());
	}
	return true;
}

bool S_Wifi::factoryTest()
{
	isFactoryTest_ = true;
	int waitResp = 0;
	//wifi resume
	if(!(int)this->resume())
	{
		ROS_INFO("%s,%d,FACTORY TEST FAIL!!",__FUNCTION__,__LINE__);
		isFactoryTest_ = false;
		return false;
	}	
	isRegDevice_ = false;
	//wifi factory test
	wifi::FactoryTestTxMsg p(0x01);
	s_wifi_tx_.push(std::move(p)).commit();
	while(!factory_test_ack_){
		usleep(20000);
		waitResp++;
		if(waitResp >= 200)//wait 4 seconds
		{
			ROS_INFO("%s,%d,FACTORY TEST FAIL!!",__FUNCTION__,__LINE__);
			isFactoryTest_ = false;
			return false;
		}
	}
	// wait for register responed
	waitResp = 0;
	ROS_INFO("INTO WIFI FACTORY TESTING!!");
	while(isRegDevice_ == false){
		usleep(20000);
		if(waitResp >= 1500){//30s
			ROS_INFO("%s,%d,FACTORY TEST FAIL!!",__FUNCTION__,__LINE__);
			isFactoryTest_ = false;
			return false;
		}
		waitResp++;
	}
	ROS_INFO("FACTORY TEST SUCCESS!!");
	isFactoryTest_ = false;
	factory_test_ack_ = false;
	return true;
}

uint8_t S_Wifi::reboot()
{
	ROS_INFO("SERIAL WIFI REBOOT!!");
	wifi::Packet p(
			-1,
			0x00,
			0x00,
			0x0B,
			{0xb7,0x7b}
			);
	s_wifi_tx_.push(std::move(p)).commit();
	return 0;
}

bool S_Wifi::resume()
{
	int resp_n = 0;
	ROS_INFO("SET WIFI RESUME!!");
	while(!is_resume_)
	{
		wifi::ResumeTxMsg p(0x00);
		s_wifi_tx_.push(std::move(p)).commit();

		usleep(20000);
		if(resp_n > 10)//200ms
			return false;
		resp_n++;
	}
	
	return true;
}

bool S_Wifi::sleep()
{
	int resp_n =0;
	ROS_INFO("SET WIFI SLEEP!!");
	while(!is_sleep_)
	{
		wifi::SuspendTxMsg p(0x00);
		s_wifi_tx_.push(std::move(p)).commit();
		usleep(500000);
		if(resp_n > 10)//5s
			return false;
		resp_n++;
	}
	return true;
}

bool S_Wifi::setWorkMode(int mode)
{
	switch (mode)
	{
		case Mode::md_idle:
			robot_work_mode_ = wifi::WorkMode::IDLE;
			break;

		case Mode::md_charge:
			robot_work_mode_ = wifi::WorkMode::CHARGE;
			break;

		case Mode::md_sleep:
			robot_work_mode_ = wifi::WorkMode::SLEEP;
			break;

		case Mode::md_remote:
			robot_work_mode_ = wifi::WorkMode::REMOTE;
			break;

		case Mode::cm_navigation:
			robot_work_mode_ = wifi::WorkMode::PLAN1;
			break;

		case Mode::cm_wall_follow:
			robot_work_mode_ = wifi::WorkMode::WALL_FOLLOW;
			break;

		case Mode::cm_spot:
			robot_work_mode_ = wifi::WorkMode::SPOT;
			break;

		case Mode::cm_exploration:
			robot_work_mode_ = wifi::WorkMode::HOMING;
			break;

		default:
			robot_work_mode_ = wifi::WorkMode::SHUTDOWN;
			break;
	}
	ROS_INFO("\033[1;35m %s,%d,set work mode %d\033[0m",__FUNCTION__,__LINE__,(int)robot_work_mode_);
	return true;
}

uint8_t S_Wifi::setSchedule(const wifi::SetScheduleRxMsg &sche)
{

	std::vector<Appointment::st_appmt> apmt_list;
	for(uint8_t i = 0;i<sche.length()/5;i++)
	{
		uint8_t schenum = sche.getScheNum(i);
		uint8_t weeks = sche.getWeek(i);
		uint8_t hours = sche.getHour(i);
		uint8_t mints = sche.getMin(i);
		uint8_t isEnable = sche.isEnable(i);

		Appointment::st_appmt apmt;
		apmt.num = schenum;
		apmt.enable = (bool)isEnable;
		apmt.hour = hours;
		apmt.mint= mints;
		apmt.week = weeks;
		apmt_list.push_back(apmt);
	}
	//--set appointment
	appmt_obj.set(apmt_list);
	return 0;
}

uint8_t S_Wifi::checkVersion()
{
	wifi::Packet p(-1 ,
				0,
				0,
				wifi::wifiVersionAckMsg::MSG_CODE,
				{0});
	s_wifi_tx_.push(std::move(p)).commit();

	return 0;
}

uint8_t S_Wifi::checkMAC()
{
	wifi::Packet pp(-1 ,
				0,
				0,
				wifi::wifiMACAckMsg::MSG_CODE,
				{0});
	s_wifi_tx_.push(std::move(pp)).commit();
	return 0;
}

void S_Wifi::taskPushBack(S_Wifi::ACT action)
{
	if(action > ACT::ACT_NONE && action < ACT::ACT_END)
	{
		MutexLock lock(&task_lock_);
		task_list_.push_back(action);
	}
}

void S_Wifi::wifi_send_routine()
{

	clock_t t;
	float period = 0;
	uint32_t upload_state_count;
	uint32_t upload_map_count;
	while( ros::ok() || wifi_quit_)
	{

		t = clock();
		if(!task_list_.empty())
		{

			pthread_mutex_lock(&task_lock_);
			S_Wifi::ACT act = task_list_.front();
			task_list_.pop_front();
			pthread_mutex_unlock(&task_lock_);
			switch(act)
			{
				case ACT::ACT_SLEEP:
					this->sleep();
					break;
				case ACT::ACT_RESUME:
					this->resume();
					break;
				case ACT::ACT_ROBOOT:
					this->reboot();
					break;
				case ACT::ACT_VERSION:
					this->checkVersion();
					break;
				case ACT::ACT_MAC:
					this->checkMAC();
					break;
				case ACT::ACT_REBIND:
					this->rebind();
					break;
				case ACT::ACT_SMART_LINK:
					this->smartLink();
					break;
				case ACT::ACT_AP_SMART_LINK:
					this->smartApLink();
					break;
				case ACT::ACT_FACTORY_TEST:
					this->factoryTest();
					break;
				case ACT::ACT_UPLOAD_MAP:
					this->uploadMap();
					break;
				case ACT::ACT_CLEAR_MAP:
					this->clearRealtimeMap(0x00);
					break;
				case ACT::ACT_UPLOAD_STATUS:
					this->uploadStatus(0xc8,0x00);
					break;
				case ACT::ACT_UPLOAD_LAST_CLEANMAP:
					this->uploadLastCleanData();
					break;
			}
			usleep(500000);
		}
		else
		{
			period = ((float)(clock() - t))/CLOCKS_PER_SEC;
			uint32_t sleep_time = 500000-(uint32_t)(period*1000000);
			if(sleep_time < 500000)
			{
				usleep(sleep_time);
			}
			else
				usleep(500000);
			if(!is_wifi_connected_)
				continue;
			upload_map_count++;
			upload_state_count++;
			if(upload_map_count >= (is_Status_Request_?3:10))
			{
				this->uploadMap();
				upload_map_count=0;
			}

			if(upload_state_count >= (is_Status_Request_?15:30))
			{
				this->uploadStatus(0xc8,0x00);
				upload_state_count=0;
			}
		}
	}
	ROS_WARN("WIFI SEND ROUTINE EXIT!");
}

void S_Wifi::cacheMapData(const Points pass_path)
{
	MutexLock lock(&map_data_lock_);
	map_data_buf_->push_back(pass_path);
}

void S_Wifi::clearMapCache()
{
	MutexLock lock(&map_data_lock_);
	map_data_buf_->clear();	
}
