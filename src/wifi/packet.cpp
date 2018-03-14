#include <cassert>
#include <cstdint>
#include <algorithm>
#include <deque>
#include <numeric>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>
//#include "log.h"
#include <ros/ros.h>
#include "wifi/packet.h"

using namespace std;

namespace wifi
{

void Packet::Parser::push(const vector<uint8_t> &data)
{
	for (auto d : data)
	{
		data_.push_back(d);
		parseFromHeader();
	}
}

void Packet::Parser::parseFromHeader()
{
	while (!data_.empty())
	{
		const auto d = data_.front();
		if (d == 0x5A)
		{
			if (!parseFromLength())
			{
				// Insufficient data
				return;
			}
		}
		else
		{
			data_.pop_front();
		}
	}
}

bool Packet::Parser::parseFromLength()
{
	if (data_.size() < 8)
	{
		// Insufficient data
		return false;
	}

	const uint16_t length = (data_[1] << 8) | data_[2];
	if (length < 8)
	{
		// Pop invalid length bytes
		data_.erase(data_.cbegin(), data_.cbegin() + 3);
		return true;
	}
	if (data_.size() < length)
	{
		// Insufficient data
		return false;
	}

	const bool is_ok = validateFromSeqNum(data_.cbegin(),
			data_.cbegin() + length);
	if (!is_ok)
	{
		data_.erase(data_.cbegin(), data_.cbegin() + length);
		return true;
	}
	// Valid packet received :D
	Packet &&p = pack(data_.cbegin(), data_.cbegin() + length);
	data_.erase(data_.cbegin(), data_.cbegin() + length);
	if (on_new_packet_)
	{
		on_new_packet_(std::move(p));
	}
	return true;
}

bool Packet::Parser::validateFromSeqNum(deque<uint8_t>::const_iterator a_beg,
		deque<uint8_t>::const_iterator a_end) const
{
	// Chksum ignores 1 header, 1 footer and 1 chksum byte
	const uint8_t calc_checksum = std::accumulate(a_beg + 1, a_end - 2, 0) & 0xFF;
	const uint8_t recv_checksum = *(a_end - 2);
	if (recv_checksum != calc_checksum)
	{
		// Invalid chksum
		ROS_WARN("wifi::Packet::Parser::validateFromSeqNum",
				"Invalid chksum, recv: %u, calc: %u", recv_checksum,
				calc_checksum);
		return false;
	}

	// Footer
	if (*(a_end - 1) != 0x5B)
	{
		ROS_WARN("wifi::Packet::Parser::validateFromSeqNum", "Invalid footer: %u",
				*(a_end - 1));
		return false;
	}

	return true;
}

Packet Packet::Parser::pack(deque<uint8_t>::const_iterator a_beg,
		deque<uint8_t>::const_iterator a_end) const
{
	/*
	if ( Log::instance()->logLevel() <= LOG_DEBUG )
	{
		stringstream s;
		s << "Packing data: " << std::hex << setfill('0');
		for ( auto it = a_beg; it != a_end; ++it )
		{
			s << setw(2) << (unsigned)*it << ' ';
		}
		ROS_DEBUG( "wifi::Packet::Parser::pack", "%s", s.str().c_str() );
	}
	*/

	auto it = a_beg;
	// Header
	++it;
	return {
		// Length
		static_cast<uint16_t>((*it++ << 8) | *it++),
		// Seq num
		*it++,
		// Additional code
		*it++,
		// Msg code
		*it++,
		// Data
		vector<uint8_t>{it, a_end - 2}
	};
}

Packet::Packet()
		: length_(0),
		  seq_num_(0),
		  add_code_(0),
		  msg_code_(0)
{}

Packet::Packet(const uint16_t a_length, const uint8_t a_seq_num,
		const uint8_t a_add_code, const uint8_t a_msg_code,
		const vector<uint8_t> &a_data)
		: length_(a_length),
		  seq_num_(a_seq_num),
		  add_code_(a_add_code),
		  msg_code_(a_msg_code),
		  data_(a_data)
{
	init();
}

Packet::Packet(const uint16_t a_length, const uint8_t a_seq_num,
		const uint8_t a_add_code, const uint8_t a_msg_code,
		vector<uint8_t> &&a_data)
		: length_(a_length),
		  seq_num_(a_seq_num),
		  add_code_(a_add_code),
		  msg_code_(a_msg_code),
		  data_(std::move(a_data))
{
	init();
}

vector<uint8_t> Packet::serialize() const
{
	const uint8_t checksum = (length_ >> 8) + (length_ & 0xFF) + seq_num_
			+ add_code_ + msg_code_
			+ std::accumulate(data_.cbegin(), data_.cend(), 0);
	vector<uint8_t> data;
	// Header
	data.push_back(0x5A);
	data.push_back(length_ >> 8);
	data.push_back(length_);
	data.push_back(seq_num_);
	data.push_back(add_code_);
	data.push_back(msg_code_);
	data.insert(data.cend(), data_.cbegin(), data_.cend());
	data.push_back(checksum);
	// Footer
	data.push_back(0x5B);
	return data;
}

void Packet::init()
{
	if (length_ == static_cast<uint16_t>(-1))
	{
		length_ = 8 + data_.size();
	}
	if (data_.size() != static_cast<size_t>(length_ - 8))
	{
		ROS_INFO("wifi::Packet::init", "Invalid length, calc: %u, data: %u",
				data_.size() + 8, length_);
		assert(data_.size() == static_cast<size_t>(length_ - 8));
	}
}

}
