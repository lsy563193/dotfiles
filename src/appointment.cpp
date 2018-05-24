#include <appointment.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <fcntl.h>
#include <serial.h>
#include <mutex_lock.h>
#include <robot_timer.h>

using namespace Appointment;

Appmt appmt_obj;

Appmt::Appmt()
{
	appointment_set_ = false;
	appointment_change_ = false;
	for(uint8_t i=0;i<10;i++)
	{
		apmt_l_.push_back({(uint8_t)(i+1),0,0,0,0});
	}
	//rw_routine(Appmt::SET);
}

bool Appmt::set(std::vector<Appointment::st_appmt> apmt_list)
{
	static int last_mint = 0;
	if(1)
	{
		MutexLock lock(&appmt_lock_);
		apmt_l_  = apmt_list;
		rw_routine(Appmt::SET);
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
			apmt_l_[i-1] = apmt;
		}
		if(1)
		{
			MutexLock lock(&appmt_lock_);
			rw_routine(Appmt::SET);
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
			apmt_l_[i-1] = apmt;
		}

		if(1)
		{
			MutexLock lock(&appmt_lock_);
			rw_routine(Appmt::SET);
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
	rw_routine(Appmt::GET);	
	return apmt_l_;
}

int8_t Appmt::rw_routine(Appmt::SG action)//read write routine
{
	//open
	const int file_len = 150;//appointment file length
	const int apt_len = 15;//one appointment length
	int fd = open(afile,O_RDWR|O_CREAT,0X644);
	if(fd < 0){
		ROS_ERROR("%s,%d,open file %s fail",__FUNCTION__,__LINE__,afile);
		return -1;
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
			sscanf(p_buf + apt_len * i,"%2u %2u %2u %2u %2u",
						(uint8_t*)&apmt_l_[i].num,
						(uint8_t*)&apmt_l_[i].enable,
						(uint8_t*)&apmt_l_[i].week,
						(uint8_t*)&apmt_l_[i].hour,
						(uint8_t*)&apmt_l_[i].mint);

			ROS_INFO("get appointment num %u enable %u weeks %u hour %u mint %u"
						,apmt_l_[i].num
						,apmt_l_[i].enable
						,apmt_l_[i].week
						,apmt_l_[i].hour
						,apmt_l_[i].mint);
		}

		appointment_change_ = false;
	}
	else if(action == Appmt::SET)//set appointment
	{

		appointment_change_ = true;
		//write to file
		st_appmt *apmt_val;
		for(int i=0;i<apmt_l_.size();i++)
		{
			apmt_val = &apmt_l_[i];
			if(apmt_val != NULL)
			{
				char tmp_buf[apt_len] = {0};
				int offset = apmt_val->num-1;
				int pos = lseek(fd,offset*apt_len,SEEK_SET);
				//ROS_INFO("offset %d",offset);

				if( pos ==-1)
				{
					ROS_ERROR("%s,%d,lseek fail %d",__FUNCTION__,__LINE__,pos);
					close(fd);
					return -1;
				}
				memset(tmp_buf,0,apt_len);
				sprintf(tmp_buf,"%2u %2u %2u %2u %2u ",apmt_val->num,(uint8_t)apmt_val->enable,apmt_val->week,apmt_val->hour,apmt_val->mint);
				int len = write(fd,tmp_buf,apt_len);
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
	}
	close(fd);
	return 0;

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
	//get appointment
	if(appointment_change_)
		this->get();
	//calculate appointment count down
	uint16_t mints[(int)apmt_l_.size()] = {0};//the appointments count down minutes
	count_down_ = 24*60*7;
	std::ostringstream msg("");
	// -- get total minutes from now
	uint16_t cur_tol_mint = (cur_wday-1) * 24 * 60 + cur_hour*60 + cur_mint;
	// -- get appointment from apmt_l
	for(int i=0;i<apmt_l_.size();i++)
	{
		if(apmt_l_[i].enable)
		{
			// -- different minutest from current time to appointment
			int16_t diff_m = (i)*24*60 + apmt_l_[i].hour*60+apmt_l_[i].mint - cur_tol_mint;
			if(diff_m<=0)//day before 
				mints[i]= 10080+diff_m;
			else // -- day after
				mints[i]= diff_m;
			if(count_down_ >= mints[i])
			{
				count_down_  = mints[i];
				appointment_set_ = true;
			}

		}
		msg<<" ("<<(i+1)<<","<<(int)mints[i]<<")";
	}
	if(appointment_set_)
		ROS_INFO("%s,%d,\033[1;40;32mcount_down=%u minutes,\033[0m  %s",__FUNCTION__,__LINE__,count_down_,msg.str().c_str());
	return count_down_;
}

void Appmt::setPlan2Bottom(uint16_t mint)
{
	uint16_t apt = appointment_set_? (mint | 0x4000):(mint | 0x8000);
	serial.setSendData(SERIAL::CTL_APPOINTMENT_H,apt>>8);
	serial.setSendData(SERIAL::CTL_APPOINTMENT_L,apt&0x00ff);
	ROS_INFO("%s,%d \033[1;40;32m set minutes count down %d,apt = 0x%x\033[0m",__FUNCTION__,__LINE__,mint,apt);
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
