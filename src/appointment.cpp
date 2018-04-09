#include <appointment.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>

using namespace Appointment;

Appmt appmt_obj;

Appmt::Appmt()
{
	//appointment_time_ = (uint32_t)ros::Time::now().toSec()/60;
	setorget_ = false;
	appointment_set_ = false;
	appointment_change_ = false;
	for(uint8_t i=0;i<10;i++)
	{
		apmt_l_.push_back({i,0,0,0,0});
	}
}

bool Appmt::set(st_appmt &apmt)
{

	MutexLock lock(&appmt_lock_);
	setorget_ = true;
	rd_routine(&apmt);
	return true;
}

bool Appmt::set(uint8_t appTime)
{
	static bool set_appointment = false;
	MutexLock lock(&appmt_lock_);
	if(appTime & 0x80 && set_appointment == false)
	{
		set_appointment = true;
		st_appmt apmt;
		apmt.enable = 1;
		apmt.hour = (appTime&0x7f)*15/60;
		apmt.mint = (appTime&0x7f)*15/60%15;
		setorget_ = true;
		for(int i = 1;i<=7;i++)
		{
			apmt.num  = i;
			apmt.week = i;
			rd_routine(&apmt);
		}
	}
	/*
	else if(!(appTime & 0x80) && set_appointment == true)
	{
		set_appointment = false;
		st_appmt apmt;
		apmt.enable = 0;
		apmt.hour = 0;
		apmt.mint = 0;
		setorget_ = true;
		for(int i = 1;i<=7;i++)
		{
			apmt.num  = i;
			apmt.week = i;
			rd_routine(&apmt);
		}
	}
	*/
	
	return true;
}

std::vector<st_appmt> Appmt::get()
{
	MutexLock lock(&appmt_lock_);
	setorget_ = false;
	rd_routine(NULL);	
	return apmt_l_;

}

int8_t Appmt::rd_routine(st_appmt *apmt_v)//read write routine
{
	st_appmt *apmt_val = (struct st_appmt*)apmt_v; 
	//open
	const int file_len = 150;
	const int col_len = 15;
	int fd = open(afile,O_RDWR|O_CREAT,0X644);
	if(fd < 0){
		ROS_ERROR("%s,%d,open file %s fail",__FUNCTION__,__LINE__,afile);
		return -1;
	}

	char buf[file_len];
	if(!setorget_)//get appointment 
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
				return -1;
			}
			else
			{
				for(int i=0;i<len;i++)
					buf[i+offset] = tmp_buf[i];	
			}
			offset = len;
		}while(len != file_len && len != 0);

		char *p_buf = buf;
		for(int i =0 ;i<apmt_l_.size();i++)
		{	
			sscanf(p_buf+col_len*(i),"%2u %2u %2u %2u %2u",
						(uint8_t*)&apmt_l_[i].num,
						(uint8_t*)&apmt_l_[i].enable,
						(uint8_t*)&apmt_l_[i].week,
						(uint8_t*)&apmt_l_[i].hour,
						(uint8_t*)&apmt_l_[i].mint);

			ROS_INFO("get appointment num %u enable %u weeks %u hour %u mint %u",apmt_l_[i].num,
						apmt_l_[i].enable,
						apmt_l_[i].week,
						apmt_l_[i].hour,
						apmt_l_[i].mint);
		}

		appointment_change_ = false;
	}
	else//set appointment
	{
		//write to file
		if(apmt_val != NULL)
		{
			appointment_change_ = true;
			char tmp_buf[col_len] = {0};
			int offset = apmt_val->num;
			
			int pos = lseek(fd,offset*col_len,SEEK_SET);
			//ROS_INFO("offset %d",offset);
			if( pos ==-1)
			{
				ROS_ERROR("%s,%d,lseek fail %d",__FUNCTION__,__LINE__,pos);
				close(fd);
			}
			memset(tmp_buf,0,col_len);
			sprintf(tmp_buf,"%2u %2u %2u %2u %2u ",apmt_val->num,apmt_val->enable,apmt_val->week,apmt_val->hour,apmt_val->mint);
			int len = write(fd,tmp_buf,col_len);
			if(len == -1)
			{
				ROS_ERROR("%s,%d,wirte fail %d ",__FUNCTION__,__LINE__,len);
				close(fd);
				return -1;
			}
			fsync(fd);
			ROS_INFO("%s,%d,set appointment success! %s",__FUNCTION__,__LINE__,tmp_buf);

		}
	}
	close(fd);
	return 0;

}

uint32_t Appmt::getLastAppointment()
{
	appointment_set_ = false;
	struct Timer::DateTime date_time;
	date_time = robot_timer.getRealTime();
	ROS_INFO("%s,%d,cur time %s",__FUNCTION__,__LINE__,
				robot_timer.asctime());

	int cur_wday = 	robot_timer.getRealTimeWeekDay();
	int cur_hour = 	date_time.hour;
	int cur_mint = 	date_time.mint;
	//get appointment
	if(appointment_change_)
		this->get();
	uint32_t mints[(int)apmt_l_.size()] = {24*60*7};
	appointment_count_ = mints[0];

	std::ostringstream msg("minutes from now:");
	for(int i=0;i<apmt_l_.size();i++)
	{
		if(apmt_l_[i].enable)
		{
			int diff_w = (i - cur_wday );
			int diff_h = (apmt_l_[i].hour - cur_hour);
			int diff_m = (apmt_l_[i].mint - cur_mint);

			if(diff_w == 0)
				if(diff_h >=0 && diff_m >=0)
					mints[i] = diff_h*60+ diff_m;
				else
					mints[i] = 7*24*60+diff_h*60+diff_m;
			else
				mints[i] = ( diff_w < 0? (7+ diff_w):diff_w) *24*60 
							+ diff_h*60
							+ diff_m;
			if(appointment_count_ > mints[i])
			{
				appointment_count_  = mints[i];
				appointment_time_ = (uint32_t)robot_timer.getRealTimeInMint();
				appointment_set_ = true;
			}
		}
		msg<<" ("<<i<<","<<(int)mints[i]<<")";
	}
	ROS_INFO("%s,%d,\033[1;40;32mappointment_count_=%u minutes,\033[0m  %s",__FUNCTION__,__LINE__,appointment_count_,msg.str().c_str());
	return appointment_count_;
}

void Appmt::setPlan2Bottom(uint32_t mint,bool appointment_set)
{
	uint16_t apt = appointment_set? (mint & 0x7fff):(mint | 0xbfff);
	serial.setSendData(SERIAL::CTL_APPOINTMENT_H,apt>>8);
	serial.setSendData(SERIAL::CTL_APPOINTMENT_L,apt&0x00ff);
	ROS_INFO("%s,%d set minutes counter %d",__FUNCTION__,__LINE__,mint);
}

