//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "state.hpp"
#include "path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"

#include <visualization_msgs/Marker.h>
//#include "move_type.hpp"


#define ROS_INFO_FL() ROS_INFO("%s,%d",__FUNCTION__, __LINE__)
#define PP_INFO() ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__)
#define PP_WARN() ROS_WARN("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__)

#define INFO_RED(X)		ROS_INFO("%s,%d,\033[1;40;31m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_GREEN(X)	ROS_INFO("%s,%d,\033[1;40;32m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_YELLOW(X)	ROS_INFO("%s,%d,\033[1;40;33m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_BLUE(X)	ROS_INFO("%s,%d,\033[1;40;34m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_PURPLE(X)	ROS_INFO("%s,%d,\033[1;40;35m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_CYAN(X)	ROS_INFO("%s,%d,\033[1;40;36m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_WHITE(X)	ROS_INFO("%s,%d,\033[1;40;37m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_BLACK(X)	ROS_INFO("%s,%d,\033[1;40;30m"#X"\033[0m",__FUNCTION__,__LINE__)

typedef struct{
		uint32_t seq{};
		Points tmp_plan_path_{};
	} PathHead;

class Paras{
public:
	Paras(bool is_left):is_left_(is_left)
{
		narrow = is_left ? 0.187 : 0.197;

		y_min = 0.0;
		y_max = is_left ? 0.3 : 0.25;

		x_min_forward = LIDAR_OFFSET_X;
		x_max_forward = is_left ? 0.3 : 0.25;
		auto y_start_forward = is_left ? 0.06: -0.06;
		auto y_end_forward = is_left ? -ROBOT_RADIUS: ROBOT_RADIUS;
		y_min_forward = std::min(y_start_forward, y_end_forward);
		y_max_forward = std::max(y_start_forward, y_end_forward);

		auto x_side_start = 0.0;
		auto x_side_end = ROBOT_RADIUS;
		x_min_side = std::min(x_side_start, x_side_end);
		x_max_side = std::max(x_side_start, x_side_end);

		auto y_side_start = 0.0;
		auto y_side_end = is_left ? narrow + 0.01 : -narrow + 0.01;
		y_min_side = std::min(y_side_start, y_side_end);
		y_max_side = std::max(y_side_start, y_side_end);

		auto y_point1_start_corner = is_left ? 0.3 : -0.3;
		auto y_point1_end_corner = is_left ? -4.0 : 4.0;
		y_min_point1_corner = std::min(y_point1_start_corner, y_point1_end_corner);
		y_max_point1_corner = std::max(y_point1_start_corner, y_point1_end_corner);

	 	auto y_point1_start = 0.0;
		auto y_point1_end = is_left ? 4.0 : -4.0;
		y_min_point1 = std::min(y_point1_start, y_point1_end);
		y_max_point1 = std::max(y_point1_start, y_point1_end);

		auto y_target_start = is_left ? ROBOT_RADIUS : -ROBOT_RADIUS;
		auto y_target_end = is_left ? 0.4 : -0.4;
		y_min_target = std::min(y_target_start, y_target_end);
		y_max_target = std::max(y_target_start, y_target_end);
	};

	bool inPoint1Range(const Vector2<double> &point, bool is_corner) const {
		if(is_corner)
			return (point.x > 0 && point.x < 4 && point.y > y_min_point1_corner && point.y < y_max_point1_corner);
		else
			return (point.x > 0 && point.x < 0.3 && point.y > y_min_point1 && point.y < y_max_point1);
	}

	bool inTargetRange(const Vector2<double> &target) {
		if (is_left_) {
			return (target.x > 0 && target.y > 0.4) ||
						 (target.x > CHASE_X && std::abs(target.y) < ROBOT_RADIUS) ||
						 (target.y < -ROBOT_RADIUS);
		} else{
			return (target.x > 0 && target.y < -0.4) ||
						 (target.x > CHASE_X && std::abs(target.y) < ROBOT_RADIUS) ||
						 (target.y > ROBOT_RADIUS);

		}
	}

	bool inForwardRange(const Vector2<double> &point) const {
		return point.x > x_min_forward && point.x < x_max_forward && point.y > y_min_forward && point.y < y_max_forward;
	}

	bool inSidedRange(const Vector2<double> &point) const {
		return point.x > x_min_side && point.x < x_max_side && point.y > y_min_side && point.y < y_max_side;
	}

	double narrow;
	bool is_left_;
	double x_min_forward;
	double x_max_forward;
	double x_min_side;
	double x_max_side;

	double y_min;
	double y_max;

	double y_min_forward;
	double y_max_forward;

	double y_min_side;
	double y_max_side;

	double y_min_point1_corner;
	double y_max_point1_corner;
	double y_min_point1;
	double y_max_point1;

	double y_min_target;
	double y_max_target;

	const double CHASE_X = 0.107;
};

class Mode:public EventHandle
{
public:
	virtual ~Mode() { };
	void run();

	virtual bool isExit();

	virtual bool isFinish();

	virtual void setNextMode(int next_mode);

	bool isInitState() const{
			return action_i_ == ac_open_gyro || action_i_ == ac_back_form_charger ||
		action_i_ == ac_open_lidar || action_i_	== ac_align || action_i_ == ac_open_slam;
	};
	int getNextMode();

//	friend IMoveType;

	enum {
		md_idle,
		md_charge,
		md_sleep,
		md_go_to_charger,
		md_remote,

		cm_navigation,
		cm_wall_follow,
		cm_spot,
		cm_exploration,

		cm_test,
	};

	int next_mode_i_;

	int action_i_{ac_null};
	enum {
		//0
		ac_null,
		ac_open_gyro,
		ac_back_form_charger,
		ac_open_lidar,
		ac_align,
		//5
		ac_open_slam,
		ac_linear,
		ac_follow_wall_left,
		ac_follow_wall_right,
		ac_turn,
		//10
		ac_back,
		ac_go_to_charger,
		ac_idle,
		ac_sleep,
		//15
		ac_charge,
		ac_movement_stay,
		ac_movement_direct_go,
		ac_pause,
		//20
		ac_exception_resume,
		ac_check_bumper,
		ac_check_vacuum,
		ac_bumper_hit_test,
	};

	bool isExceptionTriggered();

	static boost::shared_ptr<IAction> sp_action_;
	bool isNavMode()
	{
		return isNavMode_;
	}
	void setNavMode(bool set)
	{
		isNavMode_ = set;
	}

protected:
	bool isNavMode_{false};
	int mode_i_{ac_null};
private:

};

class ModeIdle:public Mode
{
public:
	ModeIdle();
	~ModeIdle() override;
	bool isExit() override;
	bool isFinish() override;
	void remoteKeyHandler(bool state_now, bool state_last);
	void remoteDirectionLeft(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteDirectionRight(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteDirectionForward(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteHome(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteSpot(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteWallFollow(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteClean(bool state_now, bool state_last) override
	{ remoteKeyHandler(state_now, state_last);}
	void remoteMax(bool state_now, bool state_last) override ;
	void remotePlan(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override;
	void chargeDetect(bool state_now, bool state_last) override ;
	void rcon(bool state_now, bool state_last) override ;

protected:
//	std::vector<Cell_t> temp_fw_cells;
private:
	void register_events(void);
	bool battery_low_{false};

	bool plan_activated_status_;

	/*---values for rcon handle---*/
	double first_time_seen_charger_;
	double last_time_seen_charger_;
};

class ModeSleep: public Mode
{
public:
	ModeSleep();
	~ModeSleep() override ;

	bool isExit() override ;
	bool isFinish() override ;

	IAction* getNextAction();

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override;
	void keyClean(bool state_now, bool state_last) override;
	void chargeDetect(bool state_now, bool state_last) override;
	void rcon(bool state_now, bool state_last) override;
	void remotePlan(bool state_now, bool state_last) override;

private:
	bool plan_activated_status_;
};

class ModeCharge: public Mode
{
public:
	ModeCharge();
	~ModeCharge() override ;

	bool isExit() override ;
	bool isFinish() override ;

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override ;
	void remotePlan(bool state_now, bool state_last) override ;

private:
	bool plan_activated_status_;
};

class ModeRemote: public Mode
{
public:
	ModeRemote();
	~ModeRemote() override ;

	bool isExit() override ;
	bool isFinish() override ;

	IAction* getNextAction();

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override ;
	void remoteDirectionLeft(bool state_now, bool state_last) override ;
	void remoteDirectionRight(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override ;

private:
	double remote_mode_time_stamp_;

};

class ModeGoToCharger: public Mode
{
public:
	ModeGoToCharger();
	~ModeGoToCharger();

	bool isExit() override;
	bool isFinish() override;

	IAction* getNextAction();

	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
//void overCurrentBrushLeft(bool state_now, bool state_last);
//void overCurrentBrushMain(bool state_now, bool state_last);
//void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);

};

class State;
class MoveTypeFollowWall;
class MoveTypeLinear;
class ACleanMode:public Mode
{
public:
	ACleanMode();
	~ACleanMode();

	State* updateState();
	bool isFinish() override ;
	bool isExit() override;
	bool isUpdateFinish();

	void setNextModeDefault();
	void genNextAction();

	void setRconPos(float cd,float dist);

	virtual bool mapMark(bool isMarkRobot = true) = 0;
	virtual bool markRealTime(){return false;};

	bool isRemoteGoHomePoint();
	void setHomePoint();
	bool estimateChargerPos(uint32_t rcon_value);
	void setRconPos(Point_t pos);

	Cells pointsGenerateCells(Points &targets);

	// For move types
	virtual bool moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt);
	virtual void moveTypeFollowWallSaveBlocks();
	virtual bool moveTypeLinearIsFinish(MoveTypeLinear *p_mt);
	virtual void moveTypeLinearSaveBlocks();

	// Handlers
	void remoteHome(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;

	// State null
	bool checkEnterNullState();
	// State init
	bool isStateInit() const
	{
		return sp_state == state_init;
	}
	virtual bool isSwitchByEventInStateInit();
	virtual bool updateActionInStateInit();
	virtual void switchInStateInit();

	// State clean
	bool isStateClean() const
	{
		return sp_state == state_clean;
	}
	virtual bool isSwitchByEventInStateClean();
	virtual bool updateActionInStateClean(){ return false;};
	virtual void switchInStateClean();

	// State go home point
	bool isStateGoHomePoint() const
	{
		return sp_state == state_go_home_point;
	}
	virtual bool checkEnterGoHomePointState();
	virtual bool isSwitchByEventInStateGoHomePoint();
	virtual bool updateActionInStateGoHomePoint();
	virtual void switchInStateGoHomePoint();

	// State go to charger
	bool isStateGoCharger() const
	{
		return sp_state == state_go_to_charger;
	}
	bool checkEnterGoCharger();
	virtual bool isSwitchByEventInStateGoToCharger(){return false;};
	virtual bool updateActionInStateGoToCharger();
	virtual void switchInStateGoToCharger();

	// State exception resume
	bool isStateExceptionResume() const
	{
		return sp_state == state_exception_resume;
	}
	bool checkEnterExceptionResumeState();
	virtual bool isSwitchByEventInStateExceptionResume();
	virtual bool updateActionInStateExceptionResume();
	virtual void switchInStateExceptionResume();

	// State temp spot
	bool isStateTmpSpot() const
	{
		return sp_state == state_spot;
	}
	virtual bool isSwitchByEventInStateSpot();
	virtual bool updateActionInStateSpot();
	virtual void switchInStateSpot(){};

	// State trapped
	bool isStateTrapped() const
	{
		return sp_state == state_trapped;
	}
	virtual bool isSwitchByEventInStateTrapped();
	virtual bool updateActionInStateTrapped();
	virtual void switchInStateTrapped();
	bool trapped_time_out_{};

	// State exploration
	bool isStateExploration() const
	{
		return sp_state == state_exploration;
	}
	virtual bool isSwitchByEventInStateExploration();
	virtual bool updateActionInStateExploration();
	virtual void switchInStateExploration();

	// State resume low battery charge
	bool isStateResumeLowBatteryCharge() const
	{
		return sp_state == state_resume_low_battery_charge;
	}
	virtual bool isSwitchByEventInStateResumeLowBatteryCharge(){return false;};
	virtual bool updateActionInStateResumeLowBatteryCharge(){};
	virtual void switchInStateResumeLowBatteryCharge(){};

	// State charge
	bool isStateCharge() const
	{
		return sp_state == state_charge;
	}
	virtual bool isSwitchByEventInStateCharge(){return false;};
	virtual bool updateActionStateCharge(){};
	virtual void switchInStateCharge(){};

	// State pause
	bool isStatePause() const
	{
		return sp_state == state_pause;
	}
	virtual bool isSwitchByEventInStatePause(){return false;};
	virtual bool updateActionInStatePause(){};
	virtual void switchInStatePause(){};

	State *sp_state{};
	State* getState() const {
		return sp_state;
	};
	void setState(State* state){
		sp_state = state;
	}

	State *state_init = new StateInit();
	State *state_clean = new StateClean();
	State *state_exception_resume = new ExceptionResume();
	State *state_exploration = new StateExploration();

	int reach_cleaned_count_{};
	Points passed_path_{};
	Points plan_path_{};
	Point_t last_{};
	bool found_temp_charger_{};
	bool in_rcon_signal_range_{};
	bool should_mark_charger_{};
	bool should_mark_temp_charger_{};
	bool found_charger_{};

	int old_dir_{};
	int new_dir_{};

	boost::shared_ptr<APathAlgorithm> clean_path_algorithm_{};
	boost::shared_ptr<GoHomePathAlgorithm> go_home_path_algorithm_{};
	GridMap clean_map_{};
	Point_t charger_pos_{};//charger postion

protected:
	std::vector<State*> sp_saved_states;
	State *state_go_home_point = new StateGoHomePoint();
	State *state_go_to_charger = new StateGoCharger();
	State *state_charge = new StateCharge();
	State *state_trapped = new StateTrapped();
	State *state_spot =  new StateSpot();
	State *state_resume_low_battery_charge = new StateResumeLowBatteryCharge();
	State *state_pause = new StatePause();

	bool low_battery_charge_{};
	bool moved_during_pause_{};
	Points home_points_{};
	Point_t start_point_{0, 0, 0};
	bool should_go_to_charger_{false};
	bool remote_go_home_point{false};
public:

	void pubPointMarkers(const std::deque<Vector2<double>> *point, std::string frame_id);
	void pubFitLineMarker(visualization_msgs::Marker fit_line_marker);
	void visualizeMarkerInit();
	void scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr& scan);
	void setCleanMapMarkers(int16_t x, int16_t y, CellState type);
	void pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path);
	void pubLineMarker(const std::vector<LineABC> *lines);
	Vector2<double> get_middle_point(const Vector2<double>& p1,const Vector2<double>& p2,const Paras& para);
	bool check_is_valid(const Vector2<double>& point, Paras& para, const sensor_msgs::LaserScan::ConstPtr & scan);
	bool check_corner(const sensor_msgs::LaserScan::ConstPtr & scan, const Paras& para);
	bool calcLidarPath(const sensor_msgs::LaserScan::ConstPtr & scan,bool is_left ,std::deque<Vector2<double>>& points);
	Vector2<double> polar_to_cartesian(double polar,int i);
	void setTempTarget(std::deque<Vector2<double>>& points, uint32_t  seq);
	void pubTmpTarget(const Point_t &point,bool is_virtual=false);

	PathHead getTempTarget();

private:
	PathHead path_head_{};
	visualization_msgs::Marker clean_markers_,bumper_markers_, clean_map_markers_;
	ros::NodeHandle clean_nh_;
	ros::Subscriber map_sub_;
	ros::Subscriber	scanLinear_sub_;
	ros::Subscriber	scanCompensate_sub_;
	ros::Subscriber lidarPoint_sub_;
	ros::Subscriber	scanOriginal_sub_;

	ros::Publisher tmp_target_pub_;
	ros::Publisher point_marker_pub_;
	ros::Publisher send_clean_map_marker_pub_;
	ros::Publisher line_marker_pub_;
	ros::Publisher line_marker_pub2_;
	ros::Publisher fit_line_marker_pub_;
	boost::mutex temp_target_mutex_;
};

class CleanModeNav:public ACleanMode
{
public:
	CleanModeNav();
	~CleanModeNav();

	bool mapMark(bool isMarkRobot = true) override ;
	bool isExit() override;

	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void remoteDirectionLeft(bool state_now, bool state_last) override ;
	void remoteDirectionRight(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;
	void remoteWallFollow(bool state_now, bool state_last) override ;
	void remoteSpot(bool state_now, bool state_last) override;
	void remoteMax(bool state_now, bool state_last) override;
//	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
	void batteryHome(bool state_now, bool state_last) override ;
//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;

//	void overCurrentSuction(bool state_now, bool state_last);

	// State init
	bool isSwitchByEventInStateInit() override;
	bool updateActionInStateInit() override;
	void switchInStateInit() override ;

	// State clean
	bool isSwitchByEventInStateClean() override;
	bool updateActionInStateClean() override;
	void switchInStateClean() override ;

	// State go home point
	bool checkEnterGoHomePointState() override;
	bool isSwitchByEventInStateGoHomePoint() override;

	// State go to charger
	bool isSwitchByEventInStateGoToCharger() override;
	void switchInStateGoToCharger() override;

	// State tmp spot
	bool checkEnterTempSpotState();
	bool checkOutOfSpot();
	bool isSwitchByEventInStateSpot() override;
	void switchInStateSpot() override;

	// State pause
	bool checkEnterPause();
	bool checkResumePause();
	bool isSwitchByEventInStatePause() override;
	bool updateActionInStatePause() override;

	// State trapped
	bool isSwitchByEventInStateTrapped() override;

	// State charge
	bool isSwitchByEventInStateCharge() override;
	bool updateActionStateCharge() override;
	void switchInStateCharge() override;

	// State resume low battery charge
	bool checkEnterResumeLowBatteryCharge();
	bool isSwitchByEventInStateResumeLowBatteryCharge() override;
	bool updateActionInStateResumeLowBatteryCharge() override;
	void switchInStateResumeLowBatteryCharge() override;

	// State exception resume
	bool isSwitchByEventInStateExceptionResume() override;

private:
	bool moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;
	bool moveTypeLinearIsFinish(MoveTypeLinear *p_mt) override;

	bool has_aligned_and_open_slam_{false};
	float paused_odom_angle_{0};
	Point_t continue_point_{};
	bool go_home_for_low_battery_{false};

protected:
//	Cells home_point_{};
public:

};

class CleanModeExploration : public ACleanMode
{
public:
	CleanModeExploration();
	~CleanModeExploration();

	bool mapMark(bool isMarkRobot = true) override;
	bool markRealTime() override;
//	bool isExit() override;
	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
//	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;

//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentSuction(bool state_now, bool state_last);
//	void printMapAndPath();
	void switchInStateInit() override;

	void switchInStateGoHomePoint() override;
	void switchInStateGoToCharger() override;

	bool moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;
};

class CleanModeFollowWall:public ACleanMode {
public:
	CleanModeFollowWall();

	~CleanModeFollowWall() override;

	bool mapMark(bool isMarkRobot = true) override;

	void keyClean(bool state_now, bool state_last) override;
	void remoteMax(bool state_now, bool state_last) override;

	void remoteClean(bool state_now, bool state_last) override;
	void switchInStateClean() override;
	bool generatePath(GridMap &map, const Point_t &curr, const int &last_dir, Points &targets);

	bool moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) override ;

	bool updateActionInStateClean()override ;

private:
 int reach_cleaned_count_save{};
};

class CleanModeSpot:public ACleanMode
{
public:
	CleanModeSpot();
	~CleanModeSpot();

	bool mapMark(bool isMarkRobot = true) override;
//	bool isExit() override;
//	void cliffAll(bool state_now, bool state_last) override;
	void remoteClean(bool state_now, bool state_last) override;
	void keyClean(bool state_now, bool state_last) override;
	void switchInStateInit() override ;
	void switchInStateSpot() override ;
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
private:

};

class CleanModeTest:public ACleanMode
{
public:
	CleanModeTest();
	~CleanModeTest();

	bool mapMark(bool isMarkRobot = true) override;

	bool isFinish() override;

	void keyClean(bool state_now, bool state_last) override ;
	void remoteMax(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;

};
#endif //PP_MODE_H_H
