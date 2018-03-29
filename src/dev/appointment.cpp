#include <appointment.h>
#include <stdlib.h>
#include <unistd.h>
#include <ros/ros.h>
#include <stdio.h>
#include <fcntl.h>

using namespace Appointment;

Appmt appmt_obj;

bool Appmt::appointment_set_ = false;

std::vector<st_appmt> Appmt::apmt_l_(10,{0,0,0,0,0});

Appmt::Appmt()
{
	apmt_l_.clear();
}

bool Appmt::set(st_appmt &apmt)
{

	appointment_set_ = true;
	rd_routine(&apmt);
	//int th_ret = pthread_create(&th_id_,NULL,rd_routine,&apmt);
	//if(th_ret !=0 )
//	{
//		ROS_ERROR("%s,%d,thread create fail %d",__FUNCTION__,__LINE__,th_ret);
//		return false;
//	}
	return true;

}

std::vector<st_appmt> Appmt::get()
{
	appointment_set_ = false;
	rd_routine(NULL);
	//int th_ret = pthread_create(&th_id_,NULL,rd_routine,NULL);
	//if(th_ret !=0 )
	//{
	//	ROS_ERROR("%s,%d,thread create fail %d",__FUNCTION__,__LINE__,th_ret);
	//	apmt_l_.clear();
	//	return apmt_l_;
	//}
	return apmt_l_;

}

void *Appmt::rd_routine(st_appmt *apmt_v)
{	
	st_appmt *apmt_val = (struct st_appmt*)apmt_v; 
	//open
	const int file_len = 150;
	const int col_len = 15;
	int fd = open(afile,O_RDWR|O_CREAT,0X700);
	if(fd < 0){
		ROS_ERROR("%s,%d,open file %s fail",__FUNCTION__,__LINE__,afile);
		//pthread_exit(NULL);
	}

	char buf[file_len];
	if(!appointment_set_)//get appointment 
	{
		//read	
		int offset = 0;
		int len = 0;
		do{
			uint8_t tmp_buf[file_len] = {0};
			len = read(fd,tmp_buf,file_len);
			if(len == -1)
			{ 
				ROS_ERROR("%s,%d,read file %d fail",__FUNCTION__,__LINE__,len);
				close(fd);
				//pthread_exit(NULL);
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
			sscanf(p_buf+col_len*i,"%d %d %d %d %d",
						(uint8_t*)&apmt_l_[i].num,
						(uint8_t*)&apmt_l_[i].enable,
						(uint8_t*)&apmt_l_[i].week,
						(uint8_t*)&apmt_l_[i].hour,
						(uint8_t*)&apmt_l_[i].mint
				  );

			ROS_INFO("num %u enable %u weeks %u hour %u mint %u",apmt_l_[i].num,
						apmt_l_[i].enable,
						apmt_l_[i].week,
						apmt_l_[i].hour,
						apmt_l_[i].mint);
		}
		ROS_INFO("%s,%d,get appointment success! ",__FUNCTION__,__LINE__);
	}
	else//set appointment
	{
		//write
		if(apmt_val != NULL)
		{
			int offset = apmt_val->num;

			int pos = lseek(fd,offset*col_len,SEEK_SET);

			if( pos ==-1)
			{
				ROS_ERROR("%s,%d,lseek fail %d fail",__FUNCTION__,__LINE__,pos);
				close(fd);
			//	pthread_exit(NULL);
			}
			memset(buf,0,file_len);

			sprintf(buf,"%2u %2u %2u %2u %2u ",apmt_val->num,apmt_val->enable,apmt_val->week,apmt_val->hour,apmt_val->mint);
			ROS_INFO("%2d %2d %2d %2d %2d \n",(int)apmt_val->num,(int)apmt_val->enable,(int)apmt_val->week,(int)apmt_val->hour,(int)apmt_val->mint);
			int len = write(fd,buf,col_len);
			if(len == -1)
			{
				ROS_ERROR("%s,%d,wirte fail %d ",__FUNCTION__,__LINE__,len);
				close(fd);
			//	pthread_exit(NULL);
			}
			fsync(fd);
			ROS_INFO("%s,%d,set appointment success! %s",__FUNCTION__,__LINE__,buf);
		}
	}
	close(fd);
	//pthread_exit(NULL);

}
