#include "wifi.h"
#include <ros/ros.h>
#include "speaker.h"
#include "mode.hpp"

#define CLOUD_DOMAIN_ID 5479
#define CLOUD_SUBDOMAIN_ID 6369
#define CLOUD_AP "Robot"
#define CLOUD_KEY "BEEE3F8925CC677AC1F3D1D9FEBC8B8632316C4B98431632E11933E1FB3C2167E223EFCC653ED3324E3EA219CCCD3E97D8242465C9327A91578901CA65015DB1C80DD4A0F45C5CC7DF1267A2FD5C00E7BD3175C2BB08BA8CFA886CCED2F70D214FB2CC88ECCA6BB1B5F4E6EE9948D424"

S_Wifi s_wifi;

S_Wifi::S_Wifi()
{
	init();
}
S_Wifi::~S_Wifi()
{
	deinit();
}

bool S_Wifi::init()
{
	INFO_BLUE(" WIFI INIT");
	s_wifi_rx_.regOnNewMsgListener<wifi::RegDeviceRequestRxMsg>(
		[&]( const wifi::RxMsg &a_msg ) 
		{
			wifi::RegDeviceTxMsg p(	wifi::RegDeviceTxMsg::CloudEnv::MAINLAND_DEV,
									{0, 0, 1},
									CLOUD_DOMAIN_ID,
									CLOUD_SUBDOMAIN_ID,
									wifi::RegDeviceTxMsg::toKeyArray(CLOUD_KEY),
									a_msg.seq_num());
			ROS_INFO( "%s,%d,Robot::initCloud, Reply RegDeviceRequestRxMsg" ,__FUNCTION__,__LINE__);
			s_wifi_tx_.push(std::move( p )).commit();
		}
	);

	s_wifi_rx_.regOnNewMsgListener<wifi::CloudConnectedNotifRxMsg>(
			[this]( const wifi::RxMsg &a_msg ) {
			/*
				if ( is_wifi_connecting_ )
				{
					is_wifi_connecting_ = false;
					speaker.play( VOICE_WIFI_CONNECTED);
				}
			*/
			INFO_BLUE(" cloud connected notification ");
			speaker.play( VOICE_WIFI_CONNECTED);
			});

	s_wifi_rx_.regOnNewMsgListener<wifi::QueryDeviceStatusRxMsg>(
			[&]( const wifi::RxMsg &a_msg ) {
				
				const wifi::QueryDeviceStatusRxMsg &msg = static_cast<const wifi::QueryDeviceStatusRxMsg&>( a_msg );
				cloudReplyStatus( msg);
				INFO_BLUE("query device status rx");
			});
/*
	s_wifi_rx_.regOnNewMsgListener<wifi::QueryConsumableStatusRxMsg>(
			[this]( const wifi::RxMsg &a_msg ) {
				const wifi::QueryConsumableStatusRxMsg &msg = static_cast<const wifi::QueryConsumableStatusRxMsg&>( a_msg );
				//cloudSendConsumableStatus( msg );
			});
*/

	return true;
}

//not complete yet
uint8_t S_Wifi::cloudReplyStatus(const wifi::RxMsg &msg)
{
	wifi::DeviceStatusReplyTxMsg p(wifi::WorkMode::SHUTDOWN,
				wifi::DeviceStatusBaseTxMsg::RoomMode::SMALL,
				wifi::DeviceStatusBaseTxMsg::CleanMode::DUST,
				(uint8_t)0x00,
				(uint8_t)0x00,
				(uint8_t)0x00,
				(uint8_t)0x00,
				(uint8_t)0x00,
				msg.seq_num());
	s_wifi_tx_.push(std::move( p )).commit();
	return 0;
}

//for tmp test
uint8_t S_Wifi::testSend()
{
	wifi::DeviceStatusReplyTxMsg p(wifi::WorkMode::SHUTDOWN,
				wifi::DeviceStatusBaseTxMsg::RoomMode::SMALL,
				wifi::DeviceStatusBaseTxMsg::CleanMode::DUST,
				(uint8_t)0x00,
				(uint8_t)0x00,
				(uint8_t)0x00,
				(uint8_t)0x00,
				(uint8_t)0x00,
				0);
	s_wifi_tx_.push(std::move( p )).commit();
	return 0;
}

bool S_Wifi::deinit()
{
	return true;
}
