//
// Created by root on 11/20/17.
//

#ifndef PP_WALL_FOLLOW_H
#define PP_WALL_FOLLOW_H


class WallFollow {
public:

	void set_base(int8_t dir, int32_t data) {
		if (dir == 0) {
			left_baseline = data;
		}
		else {
			right_baseline = data;
		}
	}

	int32_t get_base(int8_t dir) {
		if (dir == 0) {
			return left_baseline;
		}
		else {
			return right_baseline;
		}
	}

	int32_t get_adc(int8_t dir)
	{
		if (dir == 0)
		{
			return (int32_t) getLeft();
		} else
		{
			return (int32_t) getRight();
		}
	}

	void dynamic_base(uint32_t Cy)
	{
		//ROS_INFO("Run dynamic_base.");
		static int32_t Left_Sum_Value = 0, Right_Sum_Value = 0;
		static int32_t Left_Everage_Value = 0, Right_Everage_Value = 0;
		static int32_t Left_E_Counter = 0, Right_E_Counter = 0;
		static int32_t Left_Temp_Buffer = 0, Right_Temp_Buffer = 0;
		// Dynamic adjust for left wall sensor.
		Left_Temp_Buffer = get_adc(0);
		Left_Sum_Value += Left_Temp_Buffer;
		Left_E_Counter++;
		Left_Everage_Value = Left_Sum_Value / Left_E_Counter;
		double obstacle_distance_left = DBL_MAX;
		double obstacle_distance_right = DBL_MAX;

		obstacle_distance_left = lidar.getObstacleDistance(2, ROBOT_RADIUS);
		obstacle_distance_right = lidar.getObstacleDistance(3, ROBOT_RADIUS);

		if (std::abs(Left_Everage_Value - Left_Temp_Buffer) > 20 || obstacle_distance_left < (ROBOT_RADIUS + 0.30) ||
			getLeft() > 300)
		{
//		ROS_ERROR("left_reset");
			Left_Everage_Value = 0;
			Left_E_Counter = 0;
			Left_Sum_Value = 0;
			Left_Temp_Buffer = 0;
		}
		if ((uint32_t) Left_E_Counter > Cy)
		{
			// Get the wall base line for left wall sensor.
			Left_Everage_Value += get_base(0);
			if (Left_Everage_Value > 300)Left_Everage_Value = 300;//set a limit
			// Adjust the wall base line for left wall sensor.
			set_base(0, Left_Everage_Value);
//		ROS_ERROR("left_value:%d",Left_Everage_Value);
			Left_Everage_Value = 0;
			Left_E_Counter = 0;
			Left_Sum_Value = 0;
			Left_Temp_Buffer = 0;
			//ROS_INFO("Set Left Wall base value as: %d.", get_base(0));
		}

		// Dynamic adjust for right wall sensor.
		Right_Temp_Buffer = get_adc(1);
		Right_Sum_Value += Right_Temp_Buffer;
		Right_E_Counter++;
		Right_Everage_Value = Right_Sum_Value / Right_E_Counter;

		if (std::abs(Right_Everage_Value - Right_Temp_Buffer) > 20 || obstacle_distance_right < (ROBOT_RADIUS + 0.30) ||
			getRight() > 300)
		{
//		ROS_ERROR("right_reset");
			Right_Everage_Value = 0;
			Right_E_Counter = 0;
			Right_Sum_Value = 0;
			Right_Temp_Buffer = 0;
		}
		if ((uint32_t) Right_E_Counter > Cy)
		{
			// Get the wall base line for right wall sensor.
			Right_Everage_Value += get_base(1);
			if (Right_Everage_Value > 300)Right_Everage_Value = 300;//set a limit
			// Adjust the wall base line for right wall sensor.
//		ROS_ERROR("right_value:%d",Right_Everage_Value);
			set_base(1, Right_Everage_Value);
			//ROS_INFO("%s,%d:right_value: \033[31m%d\033[0m",__FUNCTION__,__LINE__,Right_Everage_Value);
			Right_Everage_Value = 0;
			Right_E_Counter = 0;
			Right_Sum_Value = 0;
			Right_Temp_Buffer = 0;
			//ROS_INFO("Set Right Wall base value as: %d.", get_base(0));
		}

	}

	int16_t getLeft() const
	{
		return left_ - left_baseline;
	}

	void setLeft(int16_t val)
	{
		left_ = val;
	}

	int16_t getRight() const
	{
		return right_ - right_baseline;
	}

	void setRight(int16_t val)
	{
		right_ = val;
	}

private:
	int16_t left_{0};
	int16_t right_{0};
	// Value for wall sensor offset.
	int16_t left_baseline = 50;
	int16_t right_baseline = 50;

};

extern WallFollow wall;

#endif //PP_WALL_FOLLOW_H
