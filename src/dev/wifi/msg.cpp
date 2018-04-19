#include <cassert>
#include <cstdint>
#include <cstdio>
#include <array>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include "wifi/packet.h"
#include "wifi/msg.h"
#include "dev.h"

using namespace std;

namespace wifi
{

RxMsg::RxMsg(Packet &&a_packet)
		: Packet(std::move(a_packet))
{}

string SetModeRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Set mode msg: " << static_cast<int>(getWorkMode());
	return ss.str();
}

string SetRoomModeRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Set room mode msg: " << static_cast<int>(getRoomMode());
	return ss.str();
}

string SetMaxCleanPowerRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Set max clean power msg: vacuum(" << (int)vacuum() << "), mop("
			<< (int)mop() << ')';
	return ss.str();
}

string RemoteControlRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Remote control msg: " << static_cast<int>(getCmd());
	return ss.str();
}

string SetScheduleRxMsg::describe() const
{
	std::ostringstream ss;
	for(uint8_t i=0 ;i<10; i++)
	{
		ss << "\nset schedule msg: num ("<< (int)getScheNum(i)<<"), enable ("<<(int)isEnable(i)<<"),weeks ("<< (int)getWeek(i)
			<< "), hour(" <<(int)getHour(i)<< "), second(" << (int)getMin(i) <<")";

	}
	return ss.str();

}

string ResetConsumableStatusRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Reset consumable status msg: side brush(" << isSideBrush()
			<< "), main brush(" << isMainBrush() << "), filter(" << isFilter()
			<< "), cloth(" << isCloth() << "), battery(" << isBattery() << ')';
	return ss.str();
}

string SyncClockRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Sync clock msg: " << setw(2) << getDay() << '/'
			<< setw(2) << getMonth() << '/' << getYear() << ' '
			<< setw(2) << getHour() << ':' << setw(2) << getMin() << ':'
			<< setw(2) << getSec();
	return ss.str();
}

string RealtimeStatusRequestRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Realtime status request msg: " << (isEnable() ? "Enable" : "Disable");
	return ss.str();
}

string SetDoNotDisturbRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Set do not disturb msg: audio(" << isEnableAudio() << ") led("
			<< isEnableLed() << ')';
	return ss.str();
}

string FactoryResetRxMsg::describe() const
{
	std::ostringstream ss;
	ss << "Factory reset msg: " << (isReset() ? "Reset" : "NULL");
	return ss.str();
}

FactoryTestTxMsg::FactoryTestTxMsg( const uint8_t a_seq_num )
		: Packet(10, a_seq_num, 0, 0x01, {0xF0, 0x0F})
{}

SmartLinkTxMsg::SmartLinkTxMsg( const uint8_t a_seq_num )
		: Packet(10, a_seq_num, 0, 0x08, {0xD8, 0x8D})
{}

SuspendTxMsg::SuspendTxMsg( const uint8_t a_seq_num )
		: Packet(10, a_seq_num, 0, MSG_CODE, {0xE9, 0x9E})
{}

ResumeTxMsg::ResumeTxMsg( const uint8_t a_seq_num )
		: Packet(10, a_seq_num, 0, MSG_CODE, {0xFA, 0xAF})
{}

SmartApLinkTxMsg::SmartApLinkTxMsg( const std::string &a_ssid,
		const uint8_t a_seq_num )
		: Packet( -1, a_seq_num, 0, MSG_CODE,
				vector<uint8_t>( a_ssid.cbegin(), a_ssid.cend() ))
{}

RegDeviceTxMsg::RegDeviceTxMsg(const CloudEnv a_cloud_env,
		const array<uint8_t, 3> &a_dev_ver,
		const uint64_t a_domain_id,
		const uint16_t a_subdomain_id,
		const array<uint8_t, 112> &a_dev_key,
		const uint8_t a_seq_num)
		: Packet(133, a_seq_num, 0, 0x07, getInitData(a_cloud_env, a_dev_ver,
				a_domain_id, a_subdomain_id, a_dev_key))
{}

array<uint8_t, 112> RegDeviceTxMsg::toKeyArray( const string &key_str )
{
	assert( key_str.length() == 224 );
	array<uint8_t, 112> product;
	uint8_t *ptr = product.data();
	for ( size_t i = 0; i < key_str.length(); i += 2 )
	{
		sscanf( key_str.c_str() + i, "%2hhx", ptr++);
	}
	return product;
}

vector<uint8_t> RegDeviceTxMsg::getInitData(const CloudEnv a_cloud_env,
		const array<uint8_t, 3> &a_dev_ver,
		const uint64_t a_domain_id,
		const uint16_t a_subdomain_id,
		const array<uint8_t, 112> &a_dev_key)
{
	vector<uint8_t> data;
	data.reserve(125);
	data.push_back(static_cast<uint8_t>(a_cloud_env));
	// 云端后台显示0-1-2 对应数据内容 H3：0x0 H2：0x0 H1：0x1 H0：0x2
	data.push_back(0);
	data.insert(data.cend(), a_dev_ver.cbegin(), a_dev_ver.cend());
	data.push_back((a_domain_id & 0xFF0000000000) >> 40);
	data.push_back((a_domain_id & 0xFF00000000) >> 32);
	data.push_back((a_domain_id & 0xFF000000) >> 24);
	data.push_back((a_domain_id & 0xFF0000) >> 16);
	data.push_back((a_domain_id & 0xFF00) >> 8);
	data.push_back(a_domain_id & 0xFF);
	data.push_back((a_subdomain_id & 0xFF00) >> 8);
	data.push_back(a_subdomain_id & 0xFF);
	data.insert(data.cend(), a_dev_key.cbegin(), a_dev_key.cend());
	return data;
}

DeviceStatusBaseTxMsg::DeviceStatusBaseTxMsg(const uint8_t a_msg_code,
		const uint8_t a_seq_num,
		const WorkMode a_work_mode,
		const RoomMode a_room_mode,
		const CleanTool clean_tool,
		const uint8_t a_vacuum_power,
		const uint8_t a_mop_power,
		const uint8_t a_battery,
		const uint8_t a_notif_mode,
		const uint8_t a_led_mode,
		const uint8_t a_error_flag)
		: Packet(17, a_seq_num, 0, a_msg_code, getInitData(a_work_mode,
				a_room_mode, clean_tool, a_vacuum_power, a_mop_power,
				a_battery, a_notif_mode, a_led_mode, a_error_flag))
{}

vector<uint8_t> DeviceStatusBaseTxMsg::getInitData(const WorkMode a_work_mode,
		const RoomMode a_room_mode,
		const CleanTool a_clean_tool,
		const uint8_t a_vacuum_power,
		const uint8_t a_mop_power,
		const uint8_t a_battery,
		const uint8_t a_notif_mode,
		const uint8_t a_led_mode,
		const uint8_t a_error_flag)
{
	vector<uint8_t> data;
	data.reserve(9);
	data.push_back(static_cast<uint8_t>(a_work_mode));
	data.push_back(static_cast<uint8_t>(a_room_mode));
	data.push_back(static_cast<uint8_t>(a_clean_tool));
	data.push_back(a_vacuum_power);
	data.push_back(a_mop_power);
	data.push_back(a_battery);
	data.push_back(a_notif_mode);
	data.push_back(a_led_mode);
	data.push_back(a_error_flag);
	return data;
}

DeviceStatusReplyTxMsg::DeviceStatusReplyTxMsg(const WorkMode a_work_mode,
		const RoomMode a_room_mode,
		const CleanTool a_clean_tool,
		const uint8_t a_vacuum_power,
		const uint8_t a_mop_power,
		const uint8_t a_battery,
		const uint8_t a_notif_mode,
		const uint8_t a_led_mode,
		const uint8_t a_error_flag,
		const uint8_t a_seq_num)
		: DeviceStatusBaseTxMsg(0x41, a_seq_num, a_work_mode, a_room_mode,
				a_clean_tool, a_vacuum_power, a_mop_power, a_battery,
				a_notif_mode, a_led_mode, a_error_flag)
{}

ScheduleStatusTxMsg::ScheduleStatusTxMsg(const vector<Schedule> &a_schedules,
		const uint8_t a_seq_num)
		: Packet(-1, a_seq_num, 0, 0x42, getInitData(a_schedules))
{}

vector<uint8_t> ScheduleStatusTxMsg::getInitData(
		const vector<Schedule> &a_schedules)
{
	vector<uint8_t> data;
	data.reserve(5 * a_schedules.size());
	for (const auto &s : a_schedules)
	{
		data.push_back(s.id_);
		data.push_back(s.is_enable_ ? 1 : 0);
		data.push_back(s.weekday_);
		data.push_back(s.hour_);
		data.push_back(s.min_);
	}
	return data;
}

ConsumableStatusTxMsg::ConsumableStatusTxMsg(const uint16_t a_work_hour,
		const uint8_t a_side_brush_hp, const uint8_t a_main_brush_hp,
		const uint8_t a_filter_hp, const uint8_t a_cloth_hp,
		const uint8_t a_battery_hp, const uint8_t a_seq_num)
		: Packet(-1, a_seq_num, 0, 0x44, getInitData(a_work_hour,
				a_side_brush_hp, a_main_brush_hp, a_filter_hp, a_cloth_hp,
				a_battery_hp))
{}

vector<uint8_t> ConsumableStatusTxMsg::getInitData(const uint16_t a_work_hour,
		const uint8_t a_side_brush_hp, const uint8_t a_main_brush_hp,
		const uint8_t a_filter_hp, const uint8_t a_cloth_hp,
		const uint8_t a_battery_hp)
{
	return {
		static_cast<uint8_t>((a_work_hour & 0xFF00) >> 8),
		static_cast<uint8_t>(a_work_hour),
		a_side_brush_hp,
		a_main_brush_hp,
		a_filter_hp,
		a_cloth_hp,
		a_battery_hp,
	};
}

SetModeTxMsg::SetModeTxMsg(const WorkMode a_work_mode, const uint8_t a_seq_num)
		: Packet(-1, a_seq_num, 0, 0x46, {static_cast<uint8_t>(a_work_mode)})
{}

MaxCleanPowerTxMsg::MaxCleanPowerTxMsg(const uint8_t a_vacuum,
		const uint8_t a_mop, const uint8_t a_seq_num)
		: Packet(-1, a_seq_num, 0, 0x48,
				getInitData(a_vacuum, a_mop))
{}

vector<uint8_t> MaxCleanPowerTxMsg::getInitData(const bool a_vacuum,
		const bool a_mop)
{
	return {
		static_cast<uint8_t>(a_vacuum ),
		static_cast<uint8_t>(a_mop ),
	};
}

DeviceStatusUploadTxMsg::DeviceStatusUploadTxMsg(const WorkMode a_work_mode,
		const RoomMode a_room_mode,
		const CleanTool a_clean_tool,
		const uint8_t a_vacuum_power,
		const uint8_t a_mop_power,
		const uint8_t a_battery,
		const uint8_t a_notif_mode,
		const uint8_t a_led_mode,
		const uint8_t a_error_flag,
		const uint8_t a_seq_num)
		: DeviceStatusBaseTxMsg(MSG_CODE, a_seq_num, a_work_mode, a_room_mode,
				a_clean_tool, a_vacuum_power, a_mop_power, a_battery,
				a_notif_mode, a_led_mode, a_error_flag)
{}

RealtimeMapUploadTxMsg::RealtimeMapUploadTxMsg(const uint32_t a_time,
		const uint8_t a_map_seq_num,
		const uint8_t a_map_packet_count,
		const std::vector<uint8_t> &a_data,
		const uint8_t a_seq_num)
		: Packet(-1, a_seq_num, 0, MSG_CODE, getInitData(a_time, a_map_seq_num,
				a_map_packet_count, a_data))
{}

vector<uint8_t> RealtimeMapUploadTxMsg::getInitData(const uint32_t a_time,
		const uint8_t a_map_seq_num,
		const uint8_t a_map_packet_count,
		const std::vector<uint8_t> &a_data)
{
	vector<uint8_t> data = {
		static_cast<uint8_t>((a_time & 0xFF000000) >> 24),
		static_cast<uint8_t>((a_time & 0xFF0000) >> 16),
		static_cast<uint8_t>((a_time & 0xFF00) >> 8),
		static_cast<uint8_t>(a_time & 0xFF),
		a_map_seq_num,
		a_map_packet_count
	};
	data.insert(data.cend(), a_data.cbegin(), a_data.cend());
	return data;
}

AccumulatedWorkTimeUploadTxMsg::AccumulatedWorkTimeUploadTxMsg(
		const uint16_t a_work_hour,
		const uint16_t a_water_tank_hour,
		const uint16_t a_dust_hour,
		const uint16_t a_mixed_hour,
		const uint8_t a_seq_num)
		: Packet(-1, a_seq_num, 0, MSG_CODE, getInitData(a_work_hour,
				a_water_tank_hour, a_dust_hour, a_mixed_hour))
{}

vector<uint8_t> AccumulatedWorkTimeUploadTxMsg::getInitData(
		const uint16_t a_work_hour, const uint16_t a_water_tank_hour,
		const uint16_t a_dust_hour, const uint16_t a_mixed_hour)
{
	return {
		static_cast<uint8_t>((a_work_hour & 0xFF00) >> 8),
		static_cast<uint8_t>(a_work_hour & 0xFF),
		static_cast<uint8_t>((a_water_tank_hour & 0xFF00) >> 8),
		static_cast<uint8_t>(a_water_tank_hour & 0xFF),
		static_cast<uint8_t>((a_dust_hour & 0xFF00) >> 8),
		static_cast<uint8_t>(a_dust_hour & 0xFF),
		static_cast<uint8_t>((a_mixed_hour & 0xFF00) >> 8),
		static_cast<uint8_t>(a_mixed_hour & 0xFF)
	};
}

ClearRealtimeMapTxMsg::ClearRealtimeMapTxMsg(const bool a_should_clear,
		const uint8_t a_seq_num)
		: Packet(-1, a_seq_num, 0, MSG_CODE, {a_should_clear})
{}

ForceUnbindTxMsg::ForceUnbindTxMsg( const uint8_t a_seq_num )
		: Packet(10, a_seq_num, 0, 0x24, {0xA6, 0x6A})
{}

}
