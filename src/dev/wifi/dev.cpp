#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <ros/ros.h>
#include "wifi/dev.h"

namespace
{

constexpr const char *TTY_PORT = "/dev/ttyS3";
constexpr int BAUDRATE = B115200;
constexpr int MAX_READ_ONCE_SIZE = 64;

}

namespace wifi
{

Dev* Dev::instance()
{
	static Dev inst;
	return &inst;
}

Dev::Dev()
		: dev_( open() )
{}

Dev::~Dev()
{
	if ( dev_ != -1 )
	{
		//Flush Serial Queue
		tcflush( dev_, TCIOFLUSH );
		close( dev_ );
	}
}

bool Dev::tx( const std::vector<uint8_t> &data ) const
{
	if ( dev_ == -1 )
	{
		return false;
	}

	size_t offset = 0;
	//ROS_INFO("tx output on (%s,115200) data = ",TTY_PORT);
	do
	{
		const int len = ::write( dev_, data.data() + offset, data.size() );
		if ( len < 0 )
		{
			ROS_WARN( "wifi::Dev::tx,Failed while write: %d", errno );
			return false;
		}
	//for tmp test
	//	for(int i=0;i<len;i++)
	//		printf("0x%x,",data[i]);
		offset += len;
	} while ( offset < data.size() );
	//std::cout<<'\n';
	return true;
}

std::vector<uint8_t> Dev::rx() const
{
	if ( dev_ == -1 )
	{
		return {};
	}

	static uint8_t buf[MAX_READ_ONCE_SIZE];
	const ssize_t len = ::read( dev_, buf, MAX_READ_ONCE_SIZE );
	if ( len < 0 )
	{
		ROS_WARN( "wifi::Dev::rx,Failed while read: %d", errno );
		return {};
	}
	return std::vector<uint8_t>(buf, buf + len);
}

int Dev::open()
{
	const int dev = ::open( TTY_PORT, O_RDWR | O_NOCTTY );
	termios s_opt;
	int err = 0;
	if ((err = tcgetattr( dev, &s_opt )) != 0)
	{
		ROS_WARN( "wifi::Dev::open,Failed while tcgetattr: %d", errno );
		close( dev );
		return -1;
	}
	if ((err = cfsetspeed( &s_opt, BAUDRATE )) != 0)
	{
		ROS_WARN( "wifi::Dev::open,Failed while cfsetspeed: %d", errno );
		close( dev );
		return -1;
	}

	//8N1
	s_opt.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
	s_opt.c_cflag |= CS8 | CREAD;
	//Disable modem static check
	s_opt.c_cflag |= CLOCAL;
	//Set Raw Mode
	cfmakeraw( &s_opt );
	if ((err = tcsetattr( dev, TCSANOW, &s_opt )) != 0)
	{
		ROS_WARN( "wifi::Dev::open,Failed while tcsetattr: %d", err );
		tcflush( dev, TCIOFLUSH );
		close( dev );
		return -1;
	}
	ROS_INFO( "wifi::Dev::open ,Everything good :D" );
	return dev;
}

}
