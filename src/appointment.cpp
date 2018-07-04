#include <appointment.h>
#include <stdlib.h>
#include <unistd.h>
#include <cerrno>
#include <ros/ros.h>
#include <fcntl.h>
#include <serial.h>
#include <mutex_lock.h>
#include <robot_timer.h>
#include "log.h"

using namespace Appointment;

Appmt appmt_obj;

Appmt::Appmt()
{
	appointment_set_ = false;
	appointment_change_ = false;
	for(uint8_t i=0;i<10;i++)
	{
		appointment_list_.push_back({(uint8_t)(i+1),0,0,0,0});
	}
	//process(Appmt::SET);
}

bool Appmt::set(std::vector<Appointment::st_appmt> apmt_list)
{
	static int last_mint = 0;
	if(1)
	{
		MutexLock lock(&appmt_lock_);
		appointment_list_  = apmt_list;
		process(Appmt::SET);
	}
	uint16_t mint = nextAppointment();
	if(mint != last_mint)
	{
		setPlan2Bottom(mint);
		last_mint = mint;
	}

	update_idle_timer_ = true;
	return true;
}

bool Appmt::set(uint8_t apTime)
{
	// -- set appointment
	if(apTime > 0x80)
	{
		st_appmt apmt;
		apmt.enable = 1;
		apmt.hour = ((apTime&0x7f)*15)/60;
		apmt.mint = ((apTime&0x7f)*15)%60;
		for(int i = 1;i<=7;i++)
		{
			apmt.num  = i;
			apmt.week = 0x01<<(i-1);
			appointment_list_[i-1] = apmt;
		}
		if(1)
		{
			MutexLock lock(&appmt_lock_);
			process(Appmt::SET);
		}

		//set appointment to bottom board
		setPlan2Bottom(nextAppointment());
		update_idle_timer_ = true;
	}
	//--cancel appointment
	else if(apTime ==  0x80 )
	{
		st_appmt apmt;
		apmt.enable = 0;
		apmt.hour = 0;
		apmt.mint = 0;
		apmt.week = 0;
		for(int i = 1;i<=7;i++)
		{
			apmt.num  = i;
			appointment_list_[i-1] = apmt;
		}

		if(1)
		{
			MutexLock lock(&appmt_lock_);
			process(Appmt::SET);
		}

		//set appointment to bottom board
		setPlan2Bottom(nextAppointment());
		update_idle_timer_ = true;
	}
	return true;
}

std::vector<st_appmt> Appmt::get()
{
	MutexLock lock(&appmt_lock_);
	process(Appmt::GET);
	return appointment_list_;
}

bool Appmt::process(Appmt::SG action)//read write routine
{
	//open
	const int file_len = 150;//appointment file length
	const int line_len = 15;//one appointment length
	int fd = open(afile,O_RDWR|O_CREAT,0x644);
	if(fd < 0){
		ROS_ERROR("%s,%d,open file %s fail,errno %d",__FUNCTION__,__LINE__,afile,errno);
		return false;
	}

	char buf[file_len];
	if(action == Appmt::GET)//get appointment
	{
		//read from file

		int offset = 0;
		int len = 0;
		do{
			uint8_t tmp_buf[file_len] = {0};
			len = read(fd,tmp_buf,file_len);
			if(len == -1)
			{
				ROS_ERROR("%s,%d,read file,%s fail, errno %d ",__FUNCTION__,__LINE__,afile,len);
				close(fd);
				return false;
			}
			else
			{
				for(int i=0;i<len;i++)
					buf[i+offset] = tmp_buf[i];
			}
			offset = len;
		}while(len != file_len && len != 0);

		char *p_buf = buf;
		for(int i =0 ;i<appointment_list_.size();i++)
		{
			sscanf(p_buf + line_len * i,"%2u %2u %2u %2u %2u\n",
						(uint8_t*)&appointment_list_[i].num,
						(uint8_t*)&appointment_list_[i].enable,
						(uint8_t*)&appointment_list_[i].week,
						(uint8_t*)&appointment_list_[i].hour,
						(uint8_t*)&appointment_list_[i].mint);

			/* INFO_NOR_CON(i<7,"get appointment %u %u %u %u %u" */
						// ,appointment_list_[i].num
						// ,appointment_list_[i].enable
						// ,appointment_list_[i].week
						// ,appointment_list_[i].hour
						/* ,appointment_list_[i].mint); */
		}

		appointment_change_ = false;
	}
	else if(action == Appmt::SET)//set appointment
	{

		appointment_change_ = true;
		//write to file
		st_appmt *apmt_val;
		for(int i=0;i<appointment_list_.size();i++)
		{
			apmt_val = &appointment_list_[i];
			if(apmt_val != NULL)
			{
				char tmp_buf[line_len] = {0};
				int offset = apmt_val->num-1;
				int pos = lseek(fd,offset*line_len,SEEK_SET);
				//ROS_INFO("offset %d",offset);

				if( pos ==-1)
				{
					ROS_ERROR("%s,%d,lseek fail %d",__FUNCTION__,__LINE__,pos);
					close(fd);
					return false;
				}
				memset(tmp_buf,0,line_len);
				sprintf(tmp_buf,"%2u %2u %2u %2u %2u\n",
							apmt_val->num,
							(uint8_t)apmt_val->enable,
							apmt_val->week,
							apmt_val->hour,
							apmt_val->mint);
				int len = write(fd,tmp_buf,line_len);
				if(len == -1)
				{
					ROS_ERROR("%s,%d,wirte fail %d ",__FUNCTION__,__LINE__,len);
					close(fd);
					return false;
				}
				fsync(fd);
				//INFO_NOR_CON(i<7,"set appointment %s",tmp_buf);

			}
		}
	}
	close(fd);
	return true;

}

uint16_t Appmt::nextAppointment()
{
	appointment_set_ = false;
	//get current time
	time_t current_time;
	robot_timer.getRealCalendarTime(current_time);
	struct tm *s_current_time = localtime(&current_time);
	int cur_wday = 	s_current_time->tm_wday == 0?7:s_current_time->tm_wday;
	int cur_hour = 	s_current_time->tm_hour;
	int cur_mint = 	s_current_time->tm_min;
	INFO_GREEN("%s,%d,current time %d,%d:%d",__FUNCTION__,__LINE__,cur_wday,cur_hour,cur_mint);
	//get appointment
	if(appointment_change_)
		this->get();
	//calculate appointment count down
	//for appointment count down minutest
	uint16_t mints[(int)appointment_list_.size()] = {0};//the appointments count down minutes

	count_down_ = 24*60*7;
	std::stringstream msg("");//for print debug msg
	// -- get current time in minutes
	uint16_t cur_tol_mint = (cur_wday-1) * 24 * 60 + cur_hour*60 + cur_mint;
	// -- cal different count down from appointments
	for(int i=0;i<appointment_list_.size();i++)
	{
		if(appointment_list_[i].enable)
		{
			// -- different minutest from current time to appointment time
			int16_t diff_m = (i*24*60 + appointment_list_[i].hour*60 + appointment_list_[i].mint) - cur_tol_mint;

			mints[i]=(diff_m<=0)?(TOTAL_MINS_A_WEEK+diff_m):diff_m;

			if(count_down_ >= mints[i])
			{
				count_down_  = mints[i];
				appointment_set_ = true;
			}

		}
		msg<<" ("<<(i+1)<<","<<(int)mints[i]<<")";
	}
	if(appointment_set_)
		INFO_GREEN("%s,%d,count_down=%u minutes\n%s",__FUNCTION__,__LINE__,count_down_,msg.str().c_str());
	return count_down_;
}

void Appmt::setPlan2Bottom(uint16_t mint)
{
	uint16_t apt = appointment_set_? (mint | 0x4000):(mint | 0x8000);
	serial.setSendData(SERIAL::CTL_APPOINTMENT_H,apt>>8);
	serial.setSendData(SERIAL::CTL_APPOINTMENT_L,apt&0x00ff);
	LOG_GREEN2("%s,%d set count down minutes %d",__FUNCTION__,__LINE__,mint);
}

void Appmt::timesUp()
{
	//--update appointment count
	setPlan2Bottom(nextAppointment());
	time_up_or_wifi_setting_ack_ = true;
}

void Appmt::resetPlanStatus(void)
{
	plan_status_ = 0;
	serial.setSendData(SERIAL::CTL_APPOINTMENT_H, 0x00);
}

void Appmt::reset()
{
	//set reset command
	appmt_obj.set(0x80);
}
