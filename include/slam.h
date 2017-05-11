//
// Created by lsy563193 on 5/10/17.
//

#ifndef PP_SLAM_H
#define PP_SLAM_H
class Slam
{
public:
	Slam();
	~Slam();
	void slam_type(int type);
	//todo
	int robot_get_home_angle(){ return 0 ;};
	void enable_map_update();
	void start(void);
	void stop(void);
	void robot_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map);
	bool is_map_ready();

private:
	int slam_type_;
	ros::ServiceClient align_cli_;
	bool	is_map_ready_;
	ros::Subscriber map_sub_;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_local_;

};

#endif //PP_SLAM_H
