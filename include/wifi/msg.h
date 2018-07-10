#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <vector>
#include "wifi/packet.h"
#include <string.h>
#include <sstream>

namespace wifi
{

enum struct WorkMode
{
	SHUTDOWN = 0,
	SLEEP,
	IDLE,
	RANDOM,
	WALL_FOLLOW,
	SPOT,
	PLAN1,
	PLAN2,
	HOMING,
	CHARGE,
	REMOTE,
	// ???
	FIND,
	MODE_NULL,
};

class RxMsg: public Packet
{
public:
	explicit RxMsg(Packet &&packet);

	/**
	 * Return the best description for this msg. Intended only for debugging and
	 * logging
	 *
	 * @return
	 */
	virtual std::string describe() const = 0;

	/**
	 * Return the expected length of this message.
	 * 0 for infinity.
	 *
	 * @return
	 */
	virtual uint16_t expectedLength() const {return 0;};

	/**
	 * Checking for message length.
	 *
	 * @return true for length matches expectation.
	 */
	bool checkLength() const;
};

class FactoryTestRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x01;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Factory test msg";
	}
};

class RegDeviceRequestRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x02;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 10;
	}

	std::string describe() const override
	{
		return "Request device registration msg";
	}
};

class wifiConnectedNotifRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x02;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "wifi connected msg";
	}
};

class wifiDisconnectedNotifRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x03;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "wifi disconnected msg";
	}
};
class CloudConnectedNotifRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x04;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Cloud connected msg";
	}
};

class CloudDisconnectedNotifRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x05;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Cloud disconnected msg";
	}
};

class QueryDeviceStatusRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x41;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	std::string describe() const override
	{
		return "Query device status msg";
	}
};

class QueryScheduleStatusRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x42;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	std::string describe() const override
	{
		return "Query schedule status msg";
	}
};

class QueryConsumableStatusRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x44;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	std::string describe() const override
	{
		return "Query consumable status msg";
	}
};

class SetModeRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x46;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	WorkMode getWorkMode() const
	{
		return static_cast<WorkMode>(data().front());
	}

	std::string describe() const override;
};

class SetRoomModeRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x47;

	enum struct RoomMode
	{
		SMALL = 0,
		MEDIUM,
		LARGE,
		END_
	};

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	RoomMode getRoomMode() const
	{
		return static_cast<RoomMode>(data().front());
	}

	std::string describe() const override;
};

class SetMaxCleanPowerRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x48;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 10;
	}

	uint8_t vacuum() const
	{
		return data().front();
	}

	uint8_t mop() const
	{
		return data()[1];
	}

	std::string describe() const override;
};

class RemoteControlRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x49;

	enum struct Cmd
	{
		NO_CMD = 0,
		FORWARD,
		BACKWARD,
		LEFT,
		RIGHT,
		STOP,
		END_
	};

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	Cmd getCmd() const
	{
		return static_cast<Cmd>(data().front());
	}

	std::string describe() const override;
};

class SetScheduleRxMsg: public RxMsg
{
public:

	enum struct week
	{
		NO_WEEK = 0,
		MON,
		TUR,
		WED,
		FUR,
		FRI,
		SAT,
		SUN
	};

	static constexpr int MSG_CODE = 0x4A;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 58;
	}

	std::string describe() const override;

	uint16_t dataLength() const
	{
		return static_cast<uint16_t>(data().size());
	}

	uint8_t getScheNum(uint8_t num) const
	{
		if(num < data().size() )
			return data()[num*5];
		else 
			return 0x00;
	}

	uint8_t isEnable(uint8_t num) const
	{
		if( num < data().size() )
			return data()[num*5+1];
		else
			return 0x00;
	}

	uint8_t getWeek(uint8_t num) const
	{
		if( num < data().size() )
		{
			return data()[num*5+2];
		}
		else
			return 0;
	}

	uint8_t getHour(uint8_t num) const
	{
		if(num < data().size() )
			return data()[num*5+3];
		else
			return 0;
	}

	uint8_t getMin(uint8_t num) const
	{
		if(num<10 && num < data().size() )
			return data()[num*5+4];
		else
			return 0;
	}

};

class ResetConsumableStatusRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x4B;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 13;
	}

	bool isSideBrush() const
	{
		return data()[0];
	}

	bool isMainBrush() const
	{
		return data()[1];
	}

	bool isFilter() const
	{
		return data()[2];
	}

	bool isCloth() const
	{
		return data()[3];
	}

	bool isBattery() const
	{
		return data()[4];
	}

	std::string describe() const override;
};

class SyncClockRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x4C;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 16;
	}

	int getYear() const
	{
		return (data()[0] << 8) | data()[1];
	}

	/**
	 * Return the month of year, starting from 1
	 *
	 * @return
	 */
	int getMonth() const
	{
		return data()[2];
	}

	/**
	 * Return the day of month, starting from 1
	 *
	 * @return
	 */
	int getDay() const
	{
		return data()[3];
	}

	int getHour() const
	{
		return data()[5];
	}

	int getMin() const
	{
		return data()[6];
	}

	int getSec() const
	{
		return data()[7];
	}

	std::string describe() const override;
};

class RealtimeStatusRequestRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x4D;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	bool isEnable() const
	{
		return data().front();
	}

	std::string describe() const override;
};

class SetDoNotDisturbRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x4E;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 10;
	}

	bool isEnableAudio() const
	{
		return data()[0];
	}

	bool isEnableLed() const
	{
		return data()[1];
	}

	std::string describe() const override;
};

class FactoryResetRxMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0x4F;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 9;
	}

	bool isReset() const
	{
		return data().front();
	}

	std::string describe() const override;
};

class FactoryTestTxMsg: public Packet
{
public:
	explicit FactoryTestTxMsg( const uint8_t seq_num = 0 );
};

class SmartLinkTxMsg: public Packet
{
public:
	explicit SmartLinkTxMsg( const uint8_t seq_num = 0 );
};

class SuspendTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0x09;

	explicit SuspendTxMsg( const uint8_t seq_num = 0 );
};

class ResumeTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0x0A;

	explicit ResumeTxMsg( const uint8_t seq_num = 0 );
};

class SmartApLinkTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0x0C;

	SmartApLinkTxMsg( const std::string &ssid, const uint8_t seq_num = 0 );
};

class RegDeviceTxMsg: public Packet
{
public:
	enum struct CloudEnv
	{
		MAINLAND_DEV = 0xFF,
		MAINLAND_PROD = 0x00,
		N_AMERICA_PROD = 0x01,
		EU_PROD = 0x02,
		APAC_PROD = 0x03,
	};

	RegDeviceTxMsg(const CloudEnv cloud_env,
			const std::array<uint8_t, 3> &dev_ver,
			const uint64_t domain_id,
			const uint16_t subdomain_id,
			const std::array<uint8_t, 112> &dev_key,
			const uint8_t seq_num = 0);

	/**
	 * Covert device key in string to byte array
	 *
	 * @param key_str
	 * @return
	 */
	static std::array<uint8_t, 112> toKeyArray( const std::string &key_str );

private:
	static std::vector<uint8_t> getInitData(const CloudEnv cloud_env,
			const std::array<uint8_t, 3> &dev_ver,
			const uint64_t domain_id,
			const uint16_t subdomain_id,
			const std::array<uint8_t, 112> &dev_key);
};

class DeviceStatusBaseTxMsg: public Packet
{
public:
	enum struct RoomMode
	{
		SMALL = 0,
		MEDIUM,
		LARGE,
	};

	enum struct CleanTool
	{
		WATER_TANK = 0,
		DUST_BOX,
		MIXED,
		MOP,
	};

	DeviceStatusBaseTxMsg(const uint8_t msg_code,
			const uint8_t seq_num,
			const WorkMode work_mode,
			const RoomMode room_mode,
			const CleanTool clean_tool,
			const uint8_t vacuum_power,
			const uint8_t mop_power,
			const uint8_t battery,
			const uint8_t notif_mode,
			const uint8_t led_mode,
			const uint8_t error_flag);

private:
	static std::vector<uint8_t> getInitData(const WorkMode work_mode,
			const RoomMode room_mode,
			const CleanTool clean_tool,
			const uint8_t vacuum_power,
			const uint8_t mop_power,
			const uint8_t battery,
			const uint8_t notif_mode,
			const uint8_t led_mode,
			const uint8_t error_flag);
};

class DeviceStatusReplyTxMsg: public DeviceStatusBaseTxMsg
{
public:
	DeviceStatusReplyTxMsg(const WorkMode work_mode,
			const RoomMode room_mode,
			const CleanTool clean_tool,
			const uint8_t vacuum_power,
			const uint8_t mop_power,
			const uint8_t battery,
			const uint8_t notif_mode,
			const uint8_t led_mode,
			const uint8_t error_flag,
			const uint8_t seq_num = 0);
};

class MaxCleanPowerTxMsg: public Packet
{
public:
	MaxCleanPowerTxMsg(const uint8_t _vacuum, const uint8_t _mop,
			const uint8_t seq_num = 0);

private:
	static std::vector<uint8_t> getInitData(const uint8_t vacuum_power,
			const uint8_t mop_power);
};

class ScheduleStatusTxMsg: public Packet
{
public:
	enum struct Weekday
	{
		MON = 0x01,
		TUE = 0x02,
		WED = 0x04,
		THUR = 0x08,
		FRI = 0x10,
		SAT = 0x20,
		SUN = 0x40,
		ONCE = 0x80,
	};

	struct Schedule
	{
		uint8_t id_;
		bool is_enable_;
		uint8_t weekday_;
		uint8_t hour_;
		uint8_t min_;

		Schedule()
				: id_(0),
				  is_enable_(false),
				  weekday_(0),
				  hour_(0),
				  min_(0)
		{}

		Schedule(const uint8_t id, const bool is_enable, const uint8_t weekday,
				const uint8_t hour, const uint8_t min)
				: id_(id), is_enable_(is_enable), weekday_(weekday), hour_(hour),
				  min_(min)
		{}

		static Schedule createDisabled(const uint8_t id)
		{
			return Schedule(id, false, 0, 0, 0);
		}
	};

	ScheduleStatusTxMsg(const std::vector<Schedule> &schedules,
			const uint8_t seq_num = 0);

private:
	static std::vector<uint8_t> getInitData(
			const std::vector<Schedule> &schedules);
};

class ConsumableStatusTxMsg: public Packet
{
public:
	ConsumableStatusTxMsg(const uint16_t work_hour, const uint8_t side_brush_hp,
			const uint8_t main_brush_hp, const uint8_t filter_hp,
			const uint8_t cloth_hp, const uint8_t battery_hp,
			const uint8_t seq_num = 0);

private:
	static std::vector<uint8_t> getInitData(const uint16_t work_hour,
			const uint8_t side_brush_hp, const uint8_t main_brush_hp,
			const uint8_t filter_hp, const uint8_t cloth_hp,
			const uint8_t battery_hp);
};

class DeviceStatusUploadTxMsg: public DeviceStatusBaseTxMsg
{
public:
	static constexpr int MSG_CODE = 0xC8;

	DeviceStatusUploadTxMsg(const WorkMode work_mode,
			const RoomMode room_mode,
			const CleanTool clean_tool,
			const uint8_t vacuum_power,
			const uint8_t mop_power,
			const uint8_t battery,
			const uint8_t notif_mode,
			const uint8_t led_mode,
			const uint8_t error_flag,
			const uint8_t seq_num = 0);
};

class DeviceStatusUploadAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = DeviceStatusUploadTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Device status ack msg";
	}
};

class CleanRecordUploadTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0xC9;

	CleanRecordUploadTxMsg(const uint32_t time,
			const uint16_t clean_time,
			const uint16_t clean_area,
			const uint16_t pack_num,
			const uint16_t pack_size,
			const std::vector<uint8_t> &data,
			const uint8_t seq_num = 0);

private:
	static std::vector<uint8_t> getInitData(const uint32_t time,
			const uint16_t clean_time,
			const uint16_t clean_area,
			const uint16_t pack_num,
			const uint16_t pack_size,
			const std::vector<uint8_t> &data);
};

class CleanRecordUploadAckMsg: public RxMsg
{
	public:
	static constexpr int MSG_CODE = CleanRecordUploadTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Upload clean record ack msg";
	}
};

class RealtimeMapUploadTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0xCA;

	RealtimeMapUploadTxMsg(const uint32_t time,
			const uint8_t map_seq_num,
			const uint8_t map_packet_count,
			const std::vector<uint8_t> &data,
			const uint8_t seq_num = 0);

private:
	static std::vector<uint8_t> getInitData(const uint32_t time,
			const uint8_t map_seq_num,
			const uint8_t map_packet_count,
			const std::vector<uint8_t> &data);
};

class RealtimeMapUploadAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = RealtimeMapUploadTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Realtime map ack msg";
	}
};

class AccumulatedWorkTimeUploadTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0xCB;

	AccumulatedWorkTimeUploadTxMsg(const uint16_t work_hour,
			const uint16_t water_tank_hour,
			const uint16_t dust_hour,
			const uint16_t mixed_hour,
			const uint8_t seq_num = 0);

private:
	static std::vector<uint8_t> getInitData(const uint16_t work_hour,
			const uint16_t water_tank_hour,
			const uint16_t dust_hour,
			const uint16_t mixed_hour);
};

class ForceUnbindTxMsg: public Packet
{
public:
	explicit ForceUnbindTxMsg( const uint8_t seq_num = 0 );
};

class AccumulatedWorkTimeUploadAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = AccumulatedWorkTimeUploadTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Accumulated work time ack msg";
	}
};

class ClearRealtimeMapTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0xCD;

	ClearRealtimeMapTxMsg(const bool should_clear,
			const uint8_t seq_num = 0);
};

class ClearRealtimeMapAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = ClearRealtimeMapTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "Clear realtime map ack msg";
	}
};

class QueryNTPTxMsg: public Packet
{
public:
	static constexpr int MSG_CODE = 0x0D;

	QueryNTPTxMsg( const uint8_t seq_num = 0 );
};

class QueryNTPAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = QueryNTPTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	int getNTPTime() const
	{
		return (data()[8] << 24) | (data()[9] << 16) | (data()[10] << 8) | data()[11];
	}

	std::string describe() const override
	{
		return "Query NTP ack msg.";
	}
};

class wifiResumeAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = ResumeTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "WIFI Resume ack";
	}
};

class wifiSuspendAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = SuspendTxMsg::MSG_CODE;

	using RxMsg::RxMsg;

	std::string describe() const override
	{
		return "WIFI Suspend ack";
	}
};

class wifiVersionAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0xFE;
	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 14;
	}

	uint32_t  getModuleVersion() const
	{
		uint32_t ver = 0;
		for(int i =0 ;i<data().size();i++)
		{
			if(i<3)
				ver |= (data().at(i) & 0x000000ff)<<((2-i)*8);
		}
		return ver;
	}

	uint32_t getCloudVersion() const
	{
		uint32_t ver = 0;
		for(int i =3 ;i<data().size();i++)
		{
			if(i>=3)
				ver |= (data().at(i) & 0x000000ff)<<((5-i)*8);
		}
		return ver;
	}

	std::string describe() const override
	{
		char buf[50];
		memset(buf,0,50);
		sprintf(buf,"wifi version 0x%06x,cloud version 0x%06x.",getModuleVersion(),getCloudVersion());
		std::ostringstream ss;
		ss<<buf;
		return ss.str();
	}
};

class wifiMACAckMsg: public RxMsg
{
public:
	static constexpr int MSG_CODE = 0xFF;

	using RxMsg::RxMsg;

	uint16_t expectedLength() const override
	{
		return 14;
	}

	uint64_t getMAC() const
	{
		uint64_t MAC_ = 0;
		for(int i = 0; i < data().size(); i++)
			MAC_ |= data().at(i)<< (8 * i);
		return MAC_;
	}

	std::string describe() const override
	{
		std::ostringstream msg;
		msg<<"WIFI MAC ADDRESS:";
		char buf[20] = {0};
		char mac[6] = {0};
		for(int i = 0; i < data().size(); i++)
		{
			mac[i] = data().at(i);
		}
		sprintf(buf,"%x %x %x %x %x %x.", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
		msg<<buf;
		return msg.str();
	}
};

}
