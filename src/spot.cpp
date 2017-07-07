/**
******************************************************************************
* @file    AI Cleaning Robot
* @author  ILife Team Dxsong
* @version V1.0
* @date    17-Nov-2011
* @brief   Random Path Cleaning Function
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "movement.h"
#include "spot.h"
#include "path_planning.h"
#include "robot.hpp"
#include "robotbase.h"
#include "core_move.h"
#include "map.h"
#include "gyro.h"
#include "wav.h"
#include "motion_manage.h"
#include "event_manager.h"
#include "slam.h"

#include <algorithm>
#include <ros/ros.h>
#include <time.h>

#ifdef Turn_Speed
#undef Turn_Speed
#define Turn_Speed  18
#endif

#define SPOT_MAX_SPEED  (20)
static SpotMovement *spot_obj = NULL;

SpotMovement::SpotMovement(float diameter = 1.0)
{
	if (spot_obj == NULL)
		spot_obj = this;
	spot_diameter_ = diameter;
	stop_point_ = {0, 0};
	near_point_ = {0, 0};
	begin_point_ = {0, 0};
	is_direct_change_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	target_.clear();
	st_ = NO_SPOT;
	spot_init_ = 0;

}

SpotMovement::~SpotMovement()
{
	if (spot_obj != NULL)
		spot_obj = NULL;
	stop_point_ = {0, 0};
	near_point_ = {0, 0};
	begin_point_ = {0, 0};
	is_direct_change_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	target_.clear();
	spot_init_ = 0;
	st_ = NO_SPOT;
}

SpotMovement *SpotMovement::instance()
{
	if (spot_obj == NULL)
		ROS_ERROR("%s,%d,spot obj should be init", __FUNCTION__, __LINE__);
	return spot_obj;
}

uint8_t SpotMovement::findNearestPoint(Point32_t ref_point)
{
	std::vector<Point32_t>::reverse_iterator rtp;
	float dist = 0.0, dist_last;
	float mindist = 1.0;
	float t_dist = FLT_MAX;
	tp_ = target_.begin();
	int i = 0, pos = 0;
	int ret = 0;
	for (rtp = target_.rbegin(); rtp != target_.rend(); ++rtp)
	{
		dist = sqrt(pow(ref_point.X - rtp->X, 2) + pow(ref_point.Y - rtp->Y, 2));
		i++;
		if (dist < t_dist)
		{
			pos = (int) (target_.size() - i);
			t_dist = dist;
			near_point_ = {rtp->X,rtp->Y};
			ret = 1;
		}
	}

	ROS_WARN("%s,%d,near point (%d,%d), t.size = %u,pos = %d", __FUNCTION__, __LINE__, 
					near_point_.X, near_point_.Y, target_.size(), pos);
	while(pos){
		printf("(%d,%d), ",tp_->X,tp_->Y);
		pos--;
		tp_++;
	}
	printf(",(%d,%d)\n",tp_->X,tp_->Y);
	ROS_WARN("%s,%d,tp_ (%d,%d)",__FUNCTION__,__LINE__,tp_->X,tp_->Y);
	return ret;
}

static void spot_motor_configure(){
	if(get_vac_mode() != Vac_Max){
		set_vac_mode(Vac_Max);
		set_vac_speed();
	}
	set_main_brush_pwm(80);
	set_side_brush_pwm(60, 60);
	wav_play(WAV_CLEANING_SPOT);

}

void SpotMovement::spotInit(float diameter, Point32_t cur_point)
{

	if ((clock() / CLOCKS_PER_SEC) % 2 == 0)
	{
		spiral_type_ = SPIRAL_LEFT_OUT;
		ROS_INFO("%s %d ,spiral left out", __FUNCTION__, __LINE__);
	} else
	{
		spiral_type_ = SPIRAL_RIGHT_OUT;
		ROS_INFO("%s ,%d spiral right out", __FUNCTION__, __LINE__);
	}
	spot_diameter_ = diameter;
	stop_point_ = {0, 0};
	near_point_ = {0, 0};
	begin_point_ = {cur_point.X, cur_point.Y};
	is_direct_change_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	target_.clear();
	if (spot_init_ == 0)
	{// init
		spot_init_ = 1;
		if (getSpotType() == CLEAN_SPOT)
		{
			spot_motor_configure();
		}
	} else
	{//deinit
		spot_init_ = 0;
		if(getSpotType() == CLEAN_SPOT){
			if(get_vac_mode() == Vac_Max){
				set_vac_mode(Vac_Normal);
				set_vac_speed();
			}
			set_main_brush_pwm(50);
			set_side_brush_pwm(30, 30);
			wav_play(WAV_CLEANING_CONTINUE);

		}
	}
}


void SpotMovement::spotDeinit()
{
	work_motor_configure();
	stop_point_ = {0, 0};
	near_point_ = {0, 0};
	begin_point_ = {0, 0};
	is_direct_change_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	target_.clear();
	//resetSpotType();
	spot_init_ = 1;//set init 1
}

void SpotMovement::setStopPoint()
{
	if (tp_ == target_.begin())
	{
		stop_point_ = {tp_->X, tp_->Y};
	} else
	{
		tp_--;
		stop_point_ = {tp_->X, tp_->Y};
	}
	ROS_WARN("%s,%d, stop point (%d,%d)", __FUNCTION__, __LINE__, stop_point_.X, stop_point_.Y);
}

uint8_t SpotMovement::changeSpiralType()
{
	if (spiral_type_ == SPIRAL_RIGHT_OUT || spiral_type_ == SPIRAL_LEFT_OUT)
	{
		sout_od_cnt_ += 1;
		if (sout_od_cnt_ >= OBS_DETECT_COUNT_MAX)
		{
			sout_od_cnt_ = 0;
			ROS_WARN("spiral obs detect count reach");
			spiral_type_ = (spiral_type_ == SPIRAL_RIGHT_OUT) ? SPIRAL_LEFT_IN : SPIRAL_RIGHT_IN;
			//generateTarget(spiral_type_, spot_diameter_, &target_, begin_point_);
		} else
		{
			spiral_type_ = (spiral_type_ == SPIRAL_RIGHT_OUT) ? SPIRAL_LEFT_OUT : SPIRAL_RIGHT_OUT;
		}
	} else if (spiral_type_ == SPIRAL_RIGHT_IN || spiral_type_ == SPIRAL_LEFT_IN)
	{
		spiral_type_ = (spiral_type_ == SPIRAL_RIGHT_IN) ? SPIRAL_LEFT_IN : SPIRAL_RIGHT_IN;
		sin_od_cnt_ += 1;
		if (sin_od_cnt_ > OBS_DETECT_COUNT_MAX)
		{
			is_direct_change_ = 0;
			sin_od_cnt_ = 0;
			is_stuck_ = 1;
		}
	}
	ROS_WARN("%s,%d,spiral change %s",__FUNCTION__, __LINE__, (spiral_type_ == 1 || spiral_type_ == 4)?"out":"in");
	return spiral_type_;
}

void SpotMovement::generateTarget(uint8_t sp_type, float diameter, std::vector<Point32_t> *target,
																	const Point32_t beginpoint)
{
	uint8_t spt = sp_type;
	int32_t x, x_l, y, y_l;
	x = x_l = beginpoint.X;
	y = y_l = beginpoint.Y;
	target->clear();
	uint16_t st_c = 1;//number of spiral count
	uint16_t st_n = (uint16_t) (diameter * 1000 / CELL_SIZE);//number of spiral
	//ROS_INFO("%s,%d,number of spiral %d",__FUNCTION__,__LINE__,st_n);
	int16_t i;
	if (spt == SPIRAL_LEFT_OUT || spt == SPIRAL_RIGHT_OUT)
	{

		while (ros::ok())
		{

			if (st_c > st_n)
			{ break; }
			if (st_c == 1)
			{
				target->push_back({x, y});
				printf("spiral out: (%d,%d)->",x,y);
			
			}

			else if ((st_c % 2) == 0)
			{//even number
				x = x_l + 1;
				x_l = x;
				for (i = 0; i < st_c; i++)
				{
					y = (spt == SPIRAL_LEFT_OUT) ? (y_l + i) : (y_l - i);
					if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("(%d,%d)->",x,y);
					}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					x = x_l - i - 1;
					//target->push_back({x,y});
				}
			} else
			{//odd number
				x = x_l - 1;
				x_l = x;
				for (i = 0; i < st_c; i++)
				{
					y = (spt == SPIRAL_LEFT_OUT) ? (y_l - i) : (y_l + i);
					if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("(%d,%d)->",x,y);
					}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					x = x_l + i + 1;
					//target->push_back({x,y});
				}
			}
			x_l = x;
			y_l = y;
			st_c += 1;
		}
		target->push_back({x, y});
		printf("(%d,%d)\n",x,y);
		tp_ = target->begin();
	} else if (spt == SPIRAL_RIGHT_IN || spt == SPIRAL_LEFT_IN)
	{

		while (ros::ok())
		{
			if (st_c > st_n)
			{ break; }
			if (st_c == 1)
			{
				target->push_back({x, y});
				printf("spiral in: (%d,%d)",x,y);
			}
			else if ((st_c % 2) == 0)
			{//even number
				y = (spt == SPIRAL_RIGHT_IN) ? (y_l + 1) : (y_l - 1);
				y_l = y;
				for (i = 0; i < st_c; i++)
				{
					x = x_l - i;
					if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("<-(%d,%d)",x,y);
					}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					y = (spt == SPIRAL_RIGHT_IN) ? (y_l - i - 1) : (y_l + i + 1);
					//target->push_back({x,y});
				}
			} else
			{ //odd number
				y = (spt == SPIRAL_RIGHT_IN) ? (y_l - 1) : (y_l + 1);
				y_l = y;
				for (i = 0; i < st_c; i++)
				{
					x = x_l + i;
					if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("<-(%d,%d)",x,y);
					}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					y = (spt == SPIRAL_RIGHT_IN) ? (y_l + i + 1) : (y_l - i - 1);
					//target->push_back({x,y});
				}
			}
			x_l = x;
			y_l = y;
			st_c += 1;
		}
		target->push_back({x, y});
		printf("<-(%d,%d)\n",x,y);
		std::reverse(target->begin(), target->end());
		tp_ = target->begin();
	}
}

int8_t SpotMovement::getNextTarget(Point32_t &next_point)
{
	int8_t ret = 0;
	SpotType spt = getSpotType();
	if (!isSpotInit() && spt != NO_SPOT)//for the first time
	{
		/*---init spot move and set begin point---*/
		if (spt == CLEAN_SPOT)
			spotInit(1.0, {map_get_x_cell(), map_get_y_cell()});
		else
			spotInit(1.0, {0, 0});
		/*---generate target ,and  set target_ ---*/
		generateTarget(spiral_type_, spot_diameter_, &target_, begin_point_);
		ROS_WARN("%s,%d , on spot init, get next point (%d %d) ", __FUNCTION__, __LINE__, tp_->X, tp_->Y);
		next_point = {cell_to_count(tp_->X), cell_to_count(tp_->Y)};
		ret = 1;
	} else if (tp_ != target_.end() && spot_init_ == 1)
	{
		if (isDirectChange())//yes bumper/obs detect
		{
			resetDirectChange();
			if (!isStuck())
			{// not stuck
				setStopPoint();
				changeSpiralType();
				next_point = {cell_to_count(tp_->X), cell_to_count(tp_->Y)};
				generateTarget(spiral_type_, spot_diameter_, &target_, begin_point_);//re_generate target
				uint8_t is_find = findNearestPoint(stop_point_);//find near_point_,  tp_
				if(!is_find)
				{
					ROS_WARN("%s,%d,not find nearest point",__FUNCTION__,__LINE__);
				}
				ROS_WARN("%s,%d , on direction change, get next point (%d %d) ", __FUNCTION__, __LINE__, near_point_.X,
								 near_point_.Y);
				ret = 1;
			} else// stuck
			{
				resetStuck();
				ROS_WARN("%s,%d , is stucked, go back to begin point (%d %d) ", __FUNCTION__, __LINE__, begin_point_.X,
								 begin_point_.Y);
				next_point = {cell_to_count(begin_point_.X), cell_to_count(begin_point_.Y)};
				spotInit(1.0, {0, 0});//clear all spot variable
				if (spt == CLEAN_SPOT)
				{
					ret = 1;
				} else
					ret = 0;
				setSpotType(NO_SPOT);
			}
		} else//no bumper/obs detect
		{
			tp_++;
			if (tp_ != target_.end())
			{
				ROS_WARN("%s,%d , get next point (%d %d) ", __FUNCTION__, __LINE__, tp_->X, tp_->Y);
				next_point = {cell_to_count(tp_->X), cell_to_count(tp_->Y)};
				ret = 1;
			} else if (tp_ == target_.end())
			{
				if (spiral_type_ == SPIRAL_RIGHT_IN || spiral_type_ == SPIRAL_LEFT_IN)
				{ //end spot
					ROS_WARN("%s,%d , spot ending, ending point (%d %d) ", __FUNCTION__, __LINE__, begin_point_.X,
									 begin_point_.Y);
					next_point = {cell_to_count(begin_point_.X), cell_to_count(begin_point_.Y)};// go back to begin point
					spotInit(1.0, {0, 0});//clear all spot variable
					if (spt == CLEAN_SPOT)
					{
						ret = 1;
					} else {
						ret = 0;
					}
					setSpotType(NO_SPOT);
				} else
				{//switch to anothor spiral type
					spiral_type_ = (spiral_type_ == SPIRAL_RIGHT_OUT) ? SPIRAL_RIGHT_IN : SPIRAL_LEFT_IN;
					generateTarget(spiral_type_, spot_diameter_, &target_, begin_point_);
					ROS_WARN("%s,%d , %s ,set spiral in, get next point (%d %d) ", __FUNCTION__, __LINE__,
									 (spiral_type_ == SPIRAL_RIGHT_OUT) ? "right in" : " left in ", tp_->X, tp_->Y);
					next_point = {cell_to_count(tp_->X), cell_to_count(tp_->Y)};
					ret = 1;
				}

			}
		}
	}
	return ret;
}

uint8_t Random_Dirt_Event(void)
{
	uint16_t Radius = 0;
	uint8_t Move_Style = 1;
	uint8_t Spot_Flag = 0;
	uint8_t OBS_Counter = 0;
	uint8_t Stunk = 0;
	uint8_t Flash_Counter = 0;
	uint16_t Watch_Counter = 0;
	uint8_t Flash_Flag = 0;
	uint8_t Motor_OC_Counter = 0;


	Move_Style = First_Round;

	reset_stop_event_status();

	check_bat_set_motors(135000, 100000, 100000);

#ifdef BLDC_INSTALL
	Set_VacMode(Vac_Max);
	Set_Vac_Speed();
#endif

	move_forward(0, 0);
	reset_wheel_step();
	reset_wall_step();
	usleep(10000);

	reset_rcon_remote();

	Motor_OC_Counter = 0;
	while (ros::ok())
	{
		usleep(10000);
		Flash_Counter++;
		if (Flash_Counter > 20)
		{
			Watch_Counter++;
			if (Watch_Counter > 1000)
			{
				set_touch();
				return 1;
			}
			Flash_Counter = 0;
			Flash_Flag = 1 - Flash_Flag;
			if (Flash_Flag)
			{
				/*do led flash */
			} else
			{
				/*do led flash*/
			}
		}

		if (remote_key(Remote_All))
		{
			//Main_Brush_PWM = MainBrush_Power;
			move_forward(30, 30);
			reset_rcon_remote();
			return 0;
		}

		/*------------------------------------------------------Virtual Wall-----------------------*/
#ifdef VIRTUAL_WALL


#endif

		/*------------------------------------------------------Check Battery-----------------------*/
		if (check_bat_set_motors(135000, 100000, 120000))
		{  //Low Battery Event
			move_forward(30, 30);
			return 0;
		}

		if (check_motor_current())
		{
			Motor_OC_Counter++;
			if (Motor_OC_Counter > 50)
			{
				Motor_OC_Counter = 0;
				//Main_Brush_PWM = MainBrush_Power;
				move_forward(30, 30);
				return 0;
			}
		} else
		{
			Motor_OC_Counter = 0;
		}
		/*------------------------------------------------------stop event-----------------------*/
		if (stop_event())
		{
			stop_brifly();
			ROS_INFO("%s %d: Stop event!", __FUNCTION__, __LINE__);
			return 1;
		}
		/*------------------------------------------------------Runing Path-----------------------*/

		switch (Move_Style)
		{
			case First_Round:
				if (get_left_wheel_step() > 6000)
				{
					move_forward(0, 0);
					reset_left_wheel_step();
					Move_Style = SPIRAL_RIGHT_OUT;
				}
				set_dir_right();
				set_wheel_speed(25, 10);

				if (get_bumper_status() || get_cliff_trig() || spot_obs_status())
				{

					if (get_bumper_status())
					{
						random_back();
					} else if (get_cliff_trig())
					{
						move_back();
					}
					stop_brifly();
					turn_left(Turn_Speed, 2500);
					move_forward(10, 10);
					Move_Style = SPIRAL_LEFT_OUT;
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				break;

			case SPIRAL_RIGHT_OUT:
				if (get_left_wheel_step() > (Radius * 3))
				{
					reset_left_wheel_step();
					if (Radius < 100)
					{
						Radius += 2;
					} else
					{
						Radius += 6;
					}
					if (Radius > 140)
					{
						Move_Style = SPIRAL_RIGHT_IN;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status())
				{

					if (get_left_wall_step() < 3000)
					{
						Stunk++;
					}
					if (get_bumper_status())
					{
						random_back();
					} else if (get_cliff_trig())
					{
						move_back();
					}
					stop_brifly();
					Move_Style = SPIRAL_LEFT_OUT;
					turn_left(Turn_Speed, 2500);
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_left_wheel_speed(SPOT_MAX_SPEED);
				set_right_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case SPIRAL_RIGHT_IN:
				if (get_left_wheel_step() > (Radius * 3))
				{
					reset_left_wheel_step();
					if (Radius < 3)
					{
						Spot_Flag = 1;
					}
					if (Radius < 100)
					{
						Radius -= 1;
					} else
					{
						Radius -= 6;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status())
				{

					if (get_left_wall_step() < 3000)
					{
						Stunk++;
					}
					if (get_bumper_status())
					{
						random_back();
					} else if (get_cliff_trig())
					{
						move_back();
					}
					stop_brifly();
					Move_Style = SPIRAL_LEFT_IN;
					turn_left(Turn_Speed, 2500);
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_left_wheel_speed(SPOT_MAX_SPEED);
				set_right_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case SPIRAL_LEFT_OUT:
				if (get_right_wheel_step() > (Radius * 3))
				{
					reset_right_wheel_step();
					if (Radius < 100)
					{
						Radius += 2;
					} else
					{
						Radius += 6;
					}
					if (Radius > 140)
					{
						Move_Style = SPIRAL_LEFT_IN;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status())
				{

					if (get_left_wall_step() < 3000)
					{
						Stunk++;
					}
					if (get_bumper_status())
					{
						random_back();
					} else if (get_cliff_trig())
					{
						move_back();
					}
					stop_brifly();
					turn_right(Turn_Speed, 2000);
					Move_Style = SPIRAL_RIGHT_OUT;
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_right_wheel_speed(SPOT_MAX_SPEED);
				set_left_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			case SPIRAL_LEFT_IN:
				if (get_right_wheel_step() > (Radius * 2))
				{
					reset_right_wheel_step();
					if (Radius < 3)
					{
						Spot_Flag = 1;
					}
					if (Radius < 100)
					{
						Radius -= 1;
					} else
					{
						Radius -= 6;
					}
				}
				if (get_bumper_status() || get_cliff_trig() || spot_obs_status())
				{

					if (get_left_wall_step() < 3000)
					{
						Stunk++;
					}
					if (get_bumper_status())
					{
						random_back();
					} else if (get_cliff_trig())
					{
						move_back();
					}
					stop_brifly();
					turn_right(Turn_Speed, 2000);
					Move_Style = SPIRAL_RIGHT_IN;
					reset_wheel_step();
					reset_wall_step();
					OBS_Counter++;
				}
				set_dir_forward();
				set_right_wheel_speed(SPOT_MAX_SPEED);
				set_left_wheel_speed((SPOT_MAX_SPEED * Radius) / (Radius + 230));

				break;

			default:
				break;

		}
		if ((OBS_Counter > 5) || (Stunk > 2))
		{
			//Main_Brush_PWM = MainBrush_Power;
			move_forward(30, 30);
			return 0;
		}
		if (Spot_Flag)
		{
			Spot_Flag = 0;
			//Main_Brush_PWM = MainBrush_Power;
			move_forward(30, 30);
			return 2;
		}
	}
	//return 2;
}


