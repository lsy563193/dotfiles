//
// Created by root on 12/5/17.
//

#ifndef PP_MOVEMENT_HPP
#define PP_MOVEMENT_HPP

#include <stdint.h>
#include "mode/mode.hpp"
#include "governor.hpp"
#include "wheel.hpp"

class IMovement: public IAction,public IGovernor{
public:
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
	virtual void run();
	virtual bool isFinish()=0;

protected:
	static Point32_t s_target_p;
	static Point32_t s_origin_p;
	static float s_pos_x;
	static float s_pos_y;
};

class MovementForward: public IMovement{
public:
//	MovementForward(Point32_t);
	MovementForward(Point32_t target, const PPTargetType& path);
//	~MovementForward(){ };
	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);
	bool isFinish();

	bool isRconStop();
	bool isOBSStop();
	bool isLidarStop();
	bool isBoundaryStop();
	bool isPassTargetStop();
	bool isNearTarget();
	bool shouldMoveBack();
	void setTarget();
	void setBaseSpeed();

	bool isCellReach();
	bool isPoseReach();

private:
	int32_t integrated_;
	int32_t base_speed_;
	uint8_t integration_cycle_;
	uint32_t tick_;
	uint8_t turn_speed_;
////	PPTargetType path_;
	float odom_x_start;
	float odom_y_start;
};

class MovementBack: public IMovement{
public:
	MovementBack();
	~MovementBack(){
		//set_wheel.speed(1, 1);
	}
	void adjustSpeed(int32_t&, int32_t&);
	bool isLidarStop();
	void setTarget();
	bool isReach();
	std::string getName()
	{
		std::string name = "MovementBack";
		return name;
	}

private:
	int counter_;
	int32_t speed_;
	float distance;
	float lidar_detect_distance;
};

class MovementTurn: public IMovement{
public:

	MovementTurn(int16_t angle);
	void adjustSpeed(int32_t&, int32_t&);
	void setTarget();
	bool isReach();
	bool shouldMoveBack();
	std::string getName()
	{
		std::string name = "MovementTurn";
		return name;
	}

private:
	uint16_t accurate_;
	uint8_t speed_;
	uint8_t stage_;
	double wait_sec_;
	double waiting_start_sec_;
	bool waiting_finished_;
	// It will use lidar points to get the turn angle for every skip_lidar_turn_angle_cnt_ times.
	uint8_t skip_lidar_turn_angle_cnt_;
};

class MovementFollowWall:public IMovement {

public:
	MovementFollowWall(Point32_t start_point, Point32_t target);

	~MovementFollowWall() { /*set_wheel.speed(0,0);*/ };

	void reset_sp_turn_count() {
		turn_count = 0;
	}

	int32_t get_sp_turn_count() {
		return turn_count;
	}

	void add_sp_turn_count() {
		turn_count++;
	}

	bool sp_turn_over(const Cell_t &curr) {
		ROS_INFO("  %s %d:?? curr(%d,%d,%d)", __FUNCTION__, __LINE__, curr.X, curr.Y, curr.TH);
		/*check if spot turn*/
		if (get_sp_turn_count() > 400) {
			reset_sp_turn_count();
			ROS_WARN("  yes! sp_turn over 400");
			return true;
		}
		return false;
	}

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed);

	bool shouldMoveBack();

	bool shouldTurn();

	bool isBlockCleared();

	bool isOverOriginLine();

	bool isNewLineReach();

	bool isClosure(uint8_t closure_cnt);

	bool isIsolate();

	bool isTimeUp();

	void setTarget();

	std::string getName() {
		std::string name = "MovementFollowWall";
		return name;
	}


private:
//Variable for checking spot turn in wall follow mode_
	volatile int32_t turn_count;
	int32_t previous_;
	uint8_t seen_charger_counter;
	int next_linear_speed = INT_MAX;
	double wall_follow_detect_distance = 0.20;
	int32_t old_same_speed = 0;
	int32_t old_diff_speed = 0;
	int turn_right_angle_factor = 15;
	int16_t wall_buffer[3] = {0};
	bool is_right_angle = false;
	double time_right_angle = 0;

	int32_t same_speed_;
	int32_t diff_speed_;
};

#endif //PP_MOVEMENT_HPP
