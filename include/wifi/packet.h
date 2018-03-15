#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace wifi
{

class Packet
{
public:
	class Parser
	{
	public:
		typedef std::function<void(Packet&&)> OnNewPacketListener;

		/**
		 * Push new data for parsing
		 *
		 * @param data
		 */
		void push(const std::vector<uint8_t> &data);

		/**
		 * Called when a new packet has successfully been parsed
		 */
		OnNewPacketListener on_new_packet_;

	private:
		void parseFromHeader();
		bool parseFromLength();
		bool validateFromSeqNum(std::deque<uint8_t>::const_iterator beg,
				std::deque<uint8_t>::const_iterator end) const;
		Packet pack(std::deque<uint8_t>::const_iterator beg,
				std::deque<uint8_t>::const_iterator end) const;

		std::deque<uint8_t> data_;
	};

	Packet();
	Packet(const uint16_t length, const uint8_t seq_num, const uint8_t add_code,
			const uint8_t msg_code, const std::vector<uint8_t> &data);
	Packet(const uint16_t length, const uint8_t seq_num, const uint8_t add_code,
			const uint8_t msg_code, std::vector<uint8_t> &&data);
	explicit Packet(Packet &&rhs) = default;
	virtual ~Packet() = default;

	/**
	 * Return if the packet is valid
	 *
	 * @return
	 */
	explicit operator bool() const
	{
		return length_;
	}

	/**
	 * Serialize the packet as a byte list, ready to be sent
	 *
	 * @return
	 */
	std::vector<uint8_t> serialize() const;

	uint16_t length() const
	{
		return length_;
	}

	uint8_t seq_num() const
	{
		return seq_num_;
	}

	void setSeqNum( const uint8_t a_v )
	{
		seq_num_ = a_v;
	}

	uint8_t add_code() const
	{
		return add_code_;
	}

	uint8_t msg_code() const
	{
		return msg_code_;
	}

	const std::vector<uint8_t>& data() const
	{
		return data_;
	}

private:
	void init();

	uint16_t length_;
	uint8_t seq_num_;
	uint8_t add_code_;
	uint8_t msg_code_;
	std::vector<uint8_t> data_;
};

}
