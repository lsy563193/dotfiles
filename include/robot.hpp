#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <pp/x900sensor.h>
#include <pp/x900ctrl.h>
#include <pp/scan_ctrl.h>
#include <rplidar_ros/SetLidar.h>
#include <vector>
#include "config.h"
#include "map.h"
#include "pose.h"
#include "serial.h"
#include "mode.hpp"
//#include "mode.hpp"
#include <string.h>

#define ON true
#define OFF false

#define  _RATE 50

extern pthread_mutex_t recev_lock;
extern pthread_cond_t  recev_cond;

extern pthread_mutex_t serial_data_ready_mtx;
extern pthread_cond_t serial_data_ready_cond;

extern bool g_pp_shutdown;

extern bool robotbase_thread_enable;
extern bool send_thread_enable;
extern bool recei_thread_enable;

extern bool robotbase_thread_kill;
extern bool recei_thread_kill;
extern bool send_thread_kill;
extern bool event_manager_thread_kill;
extern bool event_handle_thread_kill;
extern bool core_thread_kill;

class Mode;

typedef enum {
	ODOM_POSITION_ODOM_ANGLE = 0,
	SLAM_POSITION_SLAM_ANGLE,
	SLAM_POSITION_ODOM_ANGLE,
} Baselink_Frame_Type;

class robot
{
public:
	robot();
	~robot();

	static robot *instance();

	tf::TransformListener		*robot_tf_;

	// Publisher functions.
//	void pubTmpTarget(const Points &points,bool is_virtual=false);
	void robotbase_routine_cb();
	void core_thread_cb();
	void runWorkMode();
	void runTestMode();
	// Service caller functions.
	bool lidarMotorCtrl(bool switch_);
	bool slamStart(void);
	bool slamStop(void);

	void initOdomPosition();

	void resetCorrection();

	void obsAdjustCount(int count);

	//get and set function
	bool isSensorReady() const
	{
		return is_sensor_ready_;
	}

	bool isTfReady() const
	{
		return is_tf_ready_;
	}

	double getWorldPoseRadian()
	{
		return robot_rad;
	}

	float getWorldPoseX()
	{
		return static_cast<float>(robot_pos.x());
	}

	float getWorldPoseY()
	{
		return static_cast<float>(robot_pos.y());
	}

	float getWorldPoseZ()
	{
		return static_cast<float>(robot_pos.z());
	}
/*

	float getRobotCorrectionX() const
	{
		return robot_correction_x_;
	}

	float getRobotCorrectionY() const
	{
		return robot_correction_y_;
	}
*/
	bool isBatteryLow() const
	{
		return battery_low_;
	}

	void setBatterLow(bool val)
	{
		battery_low_ = val;
	}
    bool isBatteryLow2() const
	{
		return battery_low2_;
	}

	void setBatterLow2(bool val)
	{
		battery_low2_ = val;
	}
	void setTfReady(bool is_ready)
	{
		is_tf_ready_ = is_ready;
	}

	double getGyroDynamicRunTime(){
		return gyro_dynamic_run_time_;
	}

	double getGyroDynamicInterval(){
		return gyro_dynamic_interval_;
	}

	void setTempSpot(void)
	{
		temp_spot_set_ = true;
	}
	void resetTempSpot(void)
	{
		temp_spot_set_ = false;
	}
	bool isTempSpot(void)
	{
		return temp_spot_set_;
	}

	Baselink_Frame_Type getBaselinkFrameType(void)
	{
		boost::mutex::scoped_lock lock(baselink_frame_type_mutex_);
		return baselink_frame_type_;
	}

	void setBaselinkFrameType(Baselink_Frame_Type frame)
	{
		boost::mutex::scoped_lock lock(baselink_frame_type_mutex_);
		baselink_frame_type_ = frame;
		ROS_DEBUG("%s %d: Base link frame type has been reset to %d.", __FUNCTION__, __LINE__, getBaselinkFrameType());
	}

	bool isScanAllow()
	{
		return scan_ctrl_.allow_publishing?true:false;
	}

	bool pubScanCtrl(bool is_pub, bool is_force_pub = false);

	void lockScanCtrl(void);

	void unlockScanCtrl(void);

	void publishCtrlStream(void);

	void updateRobotPositionForTest();

	std::string getLidarBumperDev()
	{
		return lidar_bumper_dev_;
	}

	bool checkTilt();
	bool checkTiltToSlip();

	void setCurrent(uint16_t current)
	{
		robot_current_ = current;
	}

	uint16_t getCurrent()
	{
		return robot_current_;
	}

	boost::shared_ptr<Mode> p_mode{};

	// Return sync mode for R16 and M0.
	uint8_t getR16WorkMode()
	{
		return r16_work_mode_;
	}

	// Return Mode enum.
	int getRobotWorkMode()
	{
		return robot_work_mode_;
	}

	bool getCleanMap(GridMap& map);

	void wifiSetWaterTank();

	void wifiSetVacuum();

	bool checkLidarStuck();

	void resetSideBrushTime()
	{
		side_brush_time_ = 0;
	}
	void resetMainBrushTime()
	{
		main_brush_time_ = 0;
	}
	void resetFilterTime()
	{
		filter_time_ = 0;
	}

	uint32_t getSideBrushTime()
	{
		return side_brush_time_;
	}
	uint32_t getMainBrushTime()
	{
		return main_brush_time_;
	}
	uint32_t getFilterTime()
	{
		return filter_time_;
	}

	double getRObotActualSpeed() const{
		return robot_actual_speed_;
	};
	void updateConsumableStatus();

	void updateCleanRecord(const uint32_t &time, const uint16_t &clean_time, const float &clean_area,
						   GridMap &clean_map);

	void getCleanRecord(uint32_t &time, uint16_t &clean_time, uint16_t &clean_area, GridMap &clean_map);
private:

	uint8_t getTestMode(void);

	void loadConsumableStatus();
	uint32_t side_brush_time_{0};
	uint32_t main_brush_time_{0};
	uint32_t filter_time_{0};

	uint16_t robot_up_hour_{0};
	Baselink_Frame_Type baselink_frame_type_;
	boost::mutex baselink_frame_type_mutex_;
// Lock for odom coordinate
	boost::mutex odom_mutex_;
	// Lock for mode switching.
	boost::mutex mode_mutex_;

	// This indicates the sync mode for R16 and M0.
	uint8_t r16_work_mode_{WORK_MODE};
	// This indicates the sync mode for R16 and M0.
	int robot_work_mode_{Mode::md_idle};

	bool is_sensor_ready_{};
	bool is_tf_ready_{};

	bool temp_spot_set_{};
	bool battery_low_{false};
	bool battery_low2_{false};

	tf::Vector3	robot_pos;
	double	robot_rad;

	tf::Vector3	slam_pos{};
	double	slam_rad{};

	double robot_actual_speed_{};
	// This is for the slam correction variables.
//	tf::Vector3 correction_pos{};
//	tf::Vector3	scale_correction_pos{};
//	double correction_yaw_{};
//	double	scale_correction_yaw_;
//	tf::Vector3	slam_correction_pos;
//	double	slam_correction_yaw_;

	double gyro_dynamic_run_time_;
	double gyro_dynamic_interval_;

	ros::NodeHandle robot_nh_;

	ros::Subscriber odom_sub_;

	ros::Publisher odom_pub_;
	ros::Publisher scan_ctrl_pub_;
	ros::Publisher x900_ctrl_pub_;

	ros::ServiceClient lidar_motor_cli_;
	ros::ServiceClient start_slam_cli_;
	ros::ServiceClient end_slam_cli_;

	visualization_msgs::Marker clean_markers_,bumper_markers_, clean_map_markers_;

	tf::TransformListener		*robot_wf_tf_;
	tf::Stamped<tf::Transform>	map_pose;
	tf::Stamped<tf::Transform>	wf_map_pose;

	pp::scan_ctrl scan_ctrl_;

	std::string serial_port_;
	std::string wifi_port_;
	int baud_rate_;
	std::string lidar_bumper_dev_;

	//callback function
	void robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg);
	void odomPublish(const tf::Vector3& robot_pos, double robot_radian_);

//	void robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg);

	void setRobotActualSpeed();

	bool is_locked_scan_ctrl_{false};

	bool is_first_slip{true};

	bool is_set_anglev_offset{false};

	//for check tilt
	const double ANGLE_LIMIT{5};
	const double ANGLE_TIME_LIMIT{1};
	const double WHEEL_CLIFF_TIME_LIMIT{10000};
	double angle_tilt_time_{0};
	double wheel_tilt_time_{0};

	//for check tilt
	const double ANGLE_LIMIT_TO_SLIP{3};
	const double TIME_LIMIT_TO_SLIP{0.4};
	double tilt_time_to_slip_ = 0;
	bool is_first_tilt_to_slip_{true};

	//for check lidar stuck
	double lidar_is_covered_time_{0};

	const double ROBOT_MAX_SPEED{0.3};
	uint16_t robot_current_{0};

	struct CleanRecord{
		uint32_t time;
		uint16_t clean_time; // In second
		uint16_t clean_area; // In square meter.
		GridMap clean_map;
	};
	boost::mutex last_clean_record_mutex_;
	CleanRecord last_clean_record_;

	bool verify_ok_;
};

float cellToCount(int16_t distance);

int16_t countToCell(int32_t count);

Point_t getPosition(Baselink_Frame_Type type = SLAM_POSITION_SLAM_ANGLE);

bool isAny(Dir_t dir);

bool isPos(Dir_t dir);

bool isXAxis(Dir_t dir);

void updatePosition();

Mode *getNextMode(int next_mode_i_);

void setPosition(float x, float y);
void resetPosition();
#endif
