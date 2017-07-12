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
	targets_.clear();
	st_ = NO_SPOT;
	spot_init_ = 0;

}

SpotMovement::~SpotMovement()
{
	spotDeinit();
}

SpotMovement *SpotMovement::instance()
{
	if (spot_obj == NULL)
		ROS_ERROR("%s,%d,spot obj should be init", __FUNCTION__, __LINE__);
	return spot_obj;
}

static void spot_motor_configure()
{
	set_bldc_speed(Vac_Speed_Max);
	set_main_brush_pwm(80);
	set_side_brush_pwm(60, 60);
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
	targets_.clear();
	spot_init_ = 1;
	spot_motor_configure();
}

void SpotMovement::spotDeinit()
{
	stop_point_ = {0, 0};
	near_point_ = {0, 0};
	begin_point_ = {0, 0};
	is_direct_change_ = 0;
	is_stuck_ = 0;
	sout_od_cnt_ = 0;
	sin_od_cnt_ = 0;
	targets_.clear();
	spot_diameter_ = 0;
	spot_init_ = 0;
	if(getSpotType() == CLEAN_SPOT){
		work_motor_configure();
	}
	resetSpotType();
}

void SpotMovement::setStopPoint(Point32_t *stp)
{
	Point32_t cur_point = { map_get_x_cell(),map_get_y_cell() };
	bp_ = tp_;
	for(;tp_!=targets_.begin();--tp_)//search the bumper point,backward from curent tp_
	{
		if( cur_point.X == tp_->X && cur_point.Y == tp_->Y )
		{
			bp_ = tp_;
		}
	}

	ROS_WARN("%s,%d,bumper point (%d,%d)",__FUNCTION__,__LINE__,bp_->X,bp_->Y);
	if (bp_ == targets_.begin())
	{
		if (spiral_type_ == SPIRAL_RIGHT_OUT ){
			*stp = {bp_->X, (bp_->Y - 1)};
		}
		else if( spiral_type_ == SPIRAL_LEFT_OUT){
			*stp = {bp_->X, (bp_->Y + 1)};
		}
	}
	else
	{
		//bp_--;
		if ( ( bp_-1 )->X == bp_->X )
		{
			if (spiral_type_ == SPIRAL_RIGHT_OUT || spiral_type_ == SPIRAL_LEFT_OUT)
				*stp = { ( ( bp_->X > 0 )? bp_->X + 1: bp_->X - 1), bp_->Y };
			else if (spiral_type_ == SPIRAL_RIGHT_IN || spiral_type_ == SPIRAL_LEFT_IN)
				*stp = { ( ( bp_->X > 0 )? bp_->X - 1: bp_->X + 1), bp_->Y };
		}
		else if ( ( bp_-1 )->Y == bp_->Y)
		{
			if (spiral_type_ == SPIRAL_RIGHT_OUT || spiral_type_ == SPIRAL_LEFT_OUT)
				*stp = { bp_->X, ( ( bp_->Y > 0 )? bp_->Y + 1: bp_->Y - 1 ) };
			else if (spiral_type_ == SPIRAL_RIGHT_IN || spiral_type_ == SPIRAL_LEFT_IN)
				*stp = { bp_->X, ( ( bp_->Y > 0 )? bp_->Y - 1: bp_->Y + 1 ) };
		}
	}
	ROS_WARN("%s,%d, stop point (%d,%d)", __FUNCTION__, __LINE__, stop_point_.X, stop_point_.Y);
}

uint8_t SpotMovement::changeSpiralType()
{
	if (spiral_type_ == SPIRAL_RIGHT_OUT || spiral_type_ == SPIRAL_LEFT_OUT)
	{
		sout_od_cnt_ += 1;
		if (sout_od_cnt_ > OBS_DETECT_COUNT_MAX)
		{
			sout_od_cnt_ = 0;
			ROS_WARN("%s,%d, spiral obs detect counter reached",__FUNCTION__,__LINE__);
			spiral_type_ = (spiral_type_ == SPIRAL_RIGHT_OUT) ? SPIRAL_LEFT_IN : SPIRAL_RIGHT_IN;
		}
		else
		{
			spiral_type_ = (spiral_type_ == SPIRAL_RIGHT_OUT) ? SPIRAL_LEFT_OUT : SPIRAL_RIGHT_OUT;
		}
	}
	else if (spiral_type_ == SPIRAL_RIGHT_IN || spiral_type_ == SPIRAL_LEFT_IN)
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
	ROS_WARN("%s,%d,spiral changed %s",__FUNCTION__, __LINE__, (spiral_type_ == 1 || spiral_type_ == 4)? ((spiral_type_ == 1)?"right out":"left out"):(spiral_type_ == 2?"right in":"left in"));
	return spiral_type_;
}

uint8_t SpotMovement::getNearPoint(Point32_t ref_point)
{
	int ret = 0;
	for (tp_ = targets_.begin(); tp_ != targets_.end(); ++tp_)
	{
		if(ref_point.X == tp_->X && ref_point.Y == tp_->Y){
			ret = 1;
			near_point_ = {tp_->X, tp_->Y};
			break;
		}
	}
	if(!ret){//if not find then find near point
		float dist = 0.0;
		int pos = 0, i = 0;
		std::vector<Point32_t>::reverse_iterator rtp_;
		for (rtp_ = targets_.rbegin(); rtp_ != targets_.rend(); ++rtp_){
			dist = sqrt( pow((ref_point.X - rtp_->X), 2) + pow((ref_point.Y - rtp_->Y), 2) );
			i++;
			if(absolute(dist - 1.0) < 0.9){
				ret = 1;
				pos = (targets_.size() - i);
				near_point_ = {rtp_->X, rtp_->Y};
				break;
			}

		}
		tp_ = targets_.begin();
		while(pos){// re pointer  tp_ to the current rtp_
			pos--;
			tp_ ++;
		}
	}
	ROS_WARN("%s,%d,%s near point (%d,%d)", __FUNCTION__, __LINE__,
					(ret?"found":"not found"), tp_->X, tp_->Y);
	return ret;
}

int8_t SpotMovement::spotNextTarget(Point32_t &next_point)
{
	int8_t ret = 0;
	SpotType spt = getSpotType();
	if (!isSpotInit() && spt != NO_SPOT)//for the first time
	{
		/*---init spot move and set begin point---*/
		if (spt == CLEAN_SPOT){
			spotInit(1.0, {map_get_x_cell(), map_get_y_cell()});
			wav_play(WAV_CLEANING_SPOT);
		}
		else if( spt == NORMAL_SPOT)
			spotInit(1.0, {0, 0});
		/*---generate target ,and  set targets_ ---*/
		genTargets(spiral_type_, spot_diameter_, &targets_, begin_point_);
		ROS_WARN("%s,%d , on spot init, get next point (%d %d) ", __FUNCTION__, __LINE__, tp_->X, tp_->Y);
		next_point = {cell_to_count(tp_->X), cell_to_count(tp_->Y)};
		ret = 1;
	}
	else if (tp_ != targets_.end() && spot_init_ == 1)
	{
		if (isDirectChange())// bumper/obs detect
		{
			resetDirectChange();
			
			if (!isStuck())
			{// not stuck
				changeSpiralType();
				setStopPoint(&stop_point_);
				genTargets(spiral_type_, spot_diameter_, &targets_, begin_point_);//re_generate target
				getNearPoint(stop_point_);
				next_point = {cell_to_count(near_point_.X), cell_to_count(near_point_.Y)};
				ret = 1;
				ROS_WARN("%s,%d , on direction change, get next point (%d %d) ", __FUNCTION__, __LINE__, near_point_.X,
								 near_point_.Y);
			}
			else// stuck
			{
				resetStuck();
				ROS_WARN("%s,%d , is stucked, go back to begin point (%d %d) ", __FUNCTION__, __LINE__, begin_point_.X,
								 begin_point_.Y);
				next_point = {cell_to_count(begin_point_.X), cell_to_count(begin_point_.Y)};
				ret = (spt == CLEAN_SPOT)?1:0;
				spotDeinit();//clear all spot variable
				sleep(1);
			}
		}
		else//no bumper/obs detect
		{
			//tp_++;
			if ((tp_+1) != targets_.end())
			{
				uint8_t in_row=0,in_col=0;

				while(tp_!= targets_.end())//stright to the end of col or row
				{
					if( tp_->X == (tp_+1)->X && !in_row)
					{
						tp_++;
						in_col = 1;
					}
					else if(in_col)
					{
						break;
					}
					if( tp_->Y == (tp_+1)->Y && !in_col)
					{
						tp_++;
						in_row = 1;
					}
					else if(in_row)
					{
						break;
					}
				}

				ROS_WARN("%s,%d , get next point (%d %d) ", __FUNCTION__, __LINE__, tp_->X, tp_->Y);
				next_point = {cell_to_count(tp_->X), cell_to_count(tp_->Y)};
				ret = 1;
			}
			else if ((tp_+1) == targets_.end())
			{
				if (spiral_type_ == SPIRAL_RIGHT_IN || spiral_type_ == SPIRAL_LEFT_IN)
				{ //end spot movement
					ROS_WARN("%s,%d , spot ending, ending point (%d %d) ", __FUNCTION__, __LINE__, begin_point_.X,
									 begin_point_.Y);
					next_point = {cell_to_count(begin_point_.X), cell_to_count(begin_point_.Y)};// go back to begin point
					if (spt == CLEAN_SPOT){	ret = 1;}//clean_spot return 1
					else {ret = 0;} //normal_spot return 0
					spotDeinit();//clear all spot variable
					sleep(1);
				}
				else
				{//switch to anothor spiral type
					spiral_type_ = (spiral_type_ == SPIRAL_RIGHT_OUT) ? SPIRAL_RIGHT_IN : SPIRAL_LEFT_IN;
					genTargets(spiral_type_, spot_diameter_, &targets_, begin_point_);
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

void SpotMovement::genTargets(uint8_t sp_type, 
									float diameter, 
									std::vector<Point32_t> *target,
									const Point32_t beginpoint)
{
	uint8_t spt = sp_type;
	int32_t x, x_l, y, y_l;
	x = x_l = beginpoint.X;
	y = y_l = beginpoint.Y;
	target->clear();
	ROS_WARN("%s,%d,generate targets spiral type %s ",__FUNCTION__,__LINE__, (spiral_type_ == 1 || spiral_type_ == 4)? ((spiral_type_ == 1)?"right out":"left out"):(spiral_type_ == 2?"right in":"left in"));
	uint16_t st_c = 1;//number of spiral count
	uint16_t st_n = (uint16_t) (diameter * 1000 / CELL_SIZE);//number of spiral
	//ROS_INFO("%s,%d,number of spiral %d",__FUNCTION__,__LINE__,st_n);
	int16_t i;
	if (spt == SPIRAL_LEFT_OUT || spt == SPIRAL_RIGHT_OUT)
	{

		while (ros::ok())
		{
			if (st_c > st_n)
				break;
			if (st_c == 1)
			{
				target->push_back({x, y});
				printf("spiral %s: (%d,%d)->",(spt == SPIRAL_RIGHT_IN)?"right in":"left in",x,y);	
			}
			else if ((st_c % 2) == 0)
			{//even number
				x = x_l + 1;
				x_l = x;
				for (i = 0; i < st_c; i++)
				{
					y = (spt == SPIRAL_LEFT_OUT) ? (y_l + i) : (y_l - i);
					//if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("(%d,%d)->",x,y);
					//}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					x = x_l - i - 1;
					target->push_back({x,y});
					printf("(%d,%d)->",x,y);
				}
			}
			else
			{//odd number
				x = x_l - 1;
				x_l = x;
				for (i = 0; i < st_c; i++)
				{
					y = (spt == SPIRAL_LEFT_OUT) ? (y_l - i) : (y_l + i);
					//if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("(%d,%d)->",x,y);
					//}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					x = x_l + i + 1;
					target->push_back({x,y});
					printf("(%d,%d)->",x,y);
				}
			}
			x_l = x;
			y_l = y;
			st_c += 1;
		}
		target->push_back({x, y});
		printf("(%d,%d)\n",x,y);
		tp_ = target->begin();
	}
	else if (spt == SPIRAL_RIGHT_IN || spt == SPIRAL_LEFT_IN)
	{

		while (ros::ok())
		{
			if (st_c > st_n)
				break;
			if (st_c == 1)
			{
				target->push_back({x, y});
				printf("spiral %s: (%d,%d)",(spt == SPIRAL_RIGHT_IN)?"right in":"left in",x,y);
			}
			else if ((st_c % 2) == 0)
			{//even number
				y = (spt == SPIRAL_RIGHT_IN) ? (y_l + 1) : (y_l - 1);
				y_l = y;
				for (i = 0; i < st_c; i++)
				{
					x = x_l - i;
					//if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("<-(%d,%d)",x,y);
					//}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					y = (spt == SPIRAL_RIGHT_IN) ? (y_l - i - 1) : (y_l + i + 1);
					target->push_back({x,y});
					printf("<-(%d,%d)",x,y);
				}
			}
			else
			{ //odd number
				y = (spt == SPIRAL_RIGHT_IN) ? (y_l - 1) : (y_l + 1);
				y_l = y;
				for (i = 0; i < st_c; i++)
				{
					x = x_l + i;
					//if (i == 0 || i == (st_c - 1)){
						target->push_back({x, y});
						printf("<-(%d,%d)",x,y);
					//}
				}
				for (i = 0; i < (st_c - 1); i++)
				{
					y = (spt == SPIRAL_RIGHT_IN) ? (y_l + i + 1) : (y_l - i - 1);
					target->push_back({x,y});
					printf("<-(%d,%d)",x,y);
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


