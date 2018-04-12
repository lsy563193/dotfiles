//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "state.hpp"
//#include "path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"
#include <pthread.h>
#include <visualization_msgs/Marker.h>
#include <vacuum.h>
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

class PointSelector;

class Mode:public EventHandle
{
public:
	virtual ~Mode() { };
	void run();

	virtual bool isExit();

	virtual bool isFinish();

	virtual void setNextMode(int next_mode);

	bool isInitState() const{
			return action_i_ == ac_open_gyro || action_i_ == ac_back_from_charger ||
		action_i_ == ac_open_lidar || action_i_	== ac_align || action_i_ == ac_open_slam;
	};
	int getNextMode();

	virtual bool allowRemoteUpdatePlan()
	{
		return false;
	}

//	friend IMoveType;

	enum {
		//0
		md_idle=0,
		md_charge,
		md_sleep,
		md_go_to_charger,
		md_remote,

		//5
		cm_navigation,
		cm_wall_follow,
		cm_spot,
		cm_exploration,
		cm_test,

	};

	static int next_mode_i_;

	int action_i_{ac_null};
	enum {
		//0
		ac_null,
		ac_open_gyro,
		ac_back_from_charger,
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
		ac_pause,
		ac_remote,
		ac_exception_resume,
		ac_check_bumper,
		//20
		ac_check_vacuum,
		ac_bumper_hit_test,
		ac_desk_test,
		ac_gyro_test,
		ac_water_tank_test,
		//25
		ac_life_test,
		ac_r16_test,
	};

	virtual void genNextAction();

	bool isExceptionTriggered();

	static boost::shared_ptr<IAction> sp_action_;
	bool isNavMode()
	{
		return is_clean_mode_navigation_;
	}
	void setNavMode(bool set)
	{
		is_clean_mode_navigation_ = set;
	}

	bool isExpMode(){
		return is_clean_mode_exploration_;
	}
	void setExpMode(bool set){
		is_clean_mode_exploration_ = set;
	}

	double wall_distance;
	double wheel_cliff_triggered_time_{DBL_MAX};
	bool is_wheel_cliff_triggered{};
	int mode_i_{};

	State* sp_state{};

//	boost::shared_ptr<State> getState() const {
//		return sp_state;
//	};

//	void setState(State* state){
//		sp_state = state;
//	}

protected:
	bool is_clean_mode_navigation_{false};
	bool is_clean_mode_exploration_{false};
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
//	void lidarBumper(bool state_now, bool state_last) override;
	void remoteWifi(bool state_now, bool state_last) override;
	void remotePlan(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override;
	void chargeDetect(bool state_now, bool state_last) override ;
	void rcon(bool state_now, bool state_last) override ;

	bool allowRemoteUpdatePlan() override
	{
		return true;
	}
protected:
//	std::vector<Cell_t> temp_fw_cells;
private:
	void register_events(void);

	bool plan_activated_status_{};

	pthread_mutex_t bind_lock_;

	bool trigger_wifi_rebind_{};
	bool trigger_wifi_smart_link_{};
	bool trigger_wifi_smart_ap_link_{};

	/*---values for rcon handle---*/
	double first_time_seen_charger_{ros::Time::now().toSec()};
	double last_time_seen_charger_{first_time_seen_charger_};
	boost::shared_ptr<State> st_pause = boost::make_shared<StatePause>();
};

class ModeSleep: public Mode
{
public:
	ModeSleep();
	~ModeSleep() override ;

	bool isExit() override ;
	bool isFinish() override ;

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override;
	void keyClean(bool state_now, bool state_last) override;
	void chargeDetect(bool state_now, bool state_last) override;
	void rcon(bool state_now, bool state_last) override;
	void remotePlan(bool state_now, bool state_last) override;

private:
	boost::shared_ptr<State> st_sleep = boost::make_shared<StateSleep>();
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

	bool allowRemoteUpdatePlan() override
	{
		return true;
	}

	boost::shared_ptr<State> state_charge = boost::make_shared<StateCharge>();
	boost::shared_ptr<State> state_init = boost::make_shared<StateInit>();

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

	int getNextAction();

	// For exit event handling.
	void remoteClean(bool state_now, bool state_last) override ;
	void remoteDirectionLeft(bool state_now, bool state_last) override ;
	void remoteDirectionRight(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;
	void remoteMax(bool state_now, bool state_last) override ;
	void remoteWallFollow(bool state_now, bool state_last) override ;
	void remoteSpot(bool state_now, bool state_last) override;
	void remoteHome(bool state_now, bool state_last) override ;
	void keyClean(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;

private:
	double remote_mode_time_stamp_;
	boost::shared_ptr<State> st_clean = boost::make_shared<StateClean>();
	boost::shared_ptr<State> st_init = boost::make_shared<StateInit>();

};

class ModeGoToCharger: public Mode
{
public:
	ModeGoToCharger();
	~ModeGoToCharger();

	bool isExit() override;
	bool isFinish() override;

	int getNextAction();

	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
//void overCurrentBrushLeft(bool state_now, bool state_last);
//void overCurrentBrushMain(bool state_now, bool state_last);
//void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentVacuum(bool state_now, bool state_last);
private:
	boost::shared_ptr<State> st_go_to_charger = boost::make_shared<StateGoToCharger>();
	boost::shared_ptr<State> st_init = boost::make_shared<StateInit>();
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
	bool isStateUpdateFinish();

	void setNextModeDefault();

	bool isIsolate();
	bool isGyroDynamic();
	bool generatePath(GridMap &map, const Point_t &curr, const int &last_dir, Points &targets);

	void genNextAction();
	virtual bool mapMark() = 0;
	void setCleaned(std::deque<Cell_t> cells);
//	void setLinearCleaned();
	virtual bool markRealTime(){return false;};
	virtual bool markMapInNewCell(){return false;};

	bool isFirstTimeGoHomePoint()
	{
		return first_time_go_home_point_;
	}
	void setFirstTimeGoHomePoint(bool val)
	{
		first_time_go_home_point_ = val;
	}
	bool isRemoteGoHomePoint(){
		return remote_go_home_point;
	};
	bool isGoHomePointForLowBattery(){
		return go_home_for_low_battery_;
	}
	void setHomePoint();
	bool estimateChargerPos(uint32_t rcon_value);
	void setChargerArea(const Point_t charge_pos);
	bool checkChargerPos();
	Cells pointsGenerateCells(Points &targets);

	// For move types
	bool moveTypeNewCellIsFinish(IMoveType *p_move_type);
	bool moveTypeRealTimeIsFinish(IMoveType *p_mt);

	// Handlers
//	void remoteHome(bool state_now, bool state_last) override ;
	void cliffAll(bool state_now, bool state_last) override ;
	void robotSlip(bool state_now, bool state_last) override ;
	void overCurrentBrushMain(bool state_now, bool state_last) override;
	void overCurrentVacuum(bool state_now, bool state_last) override;

	// State init
	bool isStateInit() const
	{
		return sp_state == state_init.get();
	}
	virtual bool isSwitchByEventInStateInit();
	virtual bool updateActionInStateInit();
	virtual void switchInStateInit();

	// State clean
	bool isStateClean() const
	{
		return sp_state == state_clean.get();
	}
	virtual bool isSwitchByEventInStateClean();
	virtual bool updateActionInStateClean(){ return false;};
	virtual void switchInStateClean();

	// State go home point
	bool isStateGoHomePoint() const
	{
		return sp_state == state_go_home_point.get();
	}
	virtual bool checkEnterGoHomePointState();
	virtual bool isSwitchByEventInStateGoHomePoint();
	virtual bool updateActionInStateGoHomePoint();
	virtual void switchInStateGoHomePoint();

	// State go to charger
	bool isStateGoToCharger() const
	{
		return sp_state == state_go_to_charger.get();
	}
	bool checkEnterGoToCharger();
	virtual bool isSwitchByEventInStateGoToCharger();
	virtual bool updateActionInStateGoToCharger();
	virtual void switchInStateGoToCharger();

	// State exception resume
	bool isStateExceptionResume() const
	{
		return sp_state == state_exception_resume.get();
	}
	bool checkEnterExceptionResumeState();
	virtual bool isSwitchByEventInStateExceptionResume();
	virtual bool updateActionInStateExceptionResume();
	virtual void switchInStateExceptionResume();

	// State spot
	bool isStateSpot() const
	{
		return sp_state == state_spot.get();
	}
	virtual bool isSwitchByEventInStateSpot();
	virtual bool updateActionInStateSpot();
	virtual void switchInStateSpot(){};

	// State follow wall
	bool isStateFollowWall() const
	{
		return sp_state == state_folllow_wall.get();
	}
	virtual bool isSwitchByEventInStateFollowWall();
	virtual bool updateActionInStateFollowWall();
	virtual void switchInStateFollowWall();
	bool trapped_time_out_{};
	bool trapped_closed_or_isolate{};
	bool out_of_trapped{};

	// State exploration
	bool isStateExploration() const
	{
		return sp_state == state_exploration.get();
	}
	virtual bool isSwitchByEventInStateExploration();
	virtual bool updateActionInStateExploration();
	virtual void switchInStateExploration();

	// State resume low battery charge
	bool isStateResumeLowBatteryCharge() const
	{
		return sp_state == state_resume_low_battery_charge.get();
	}
	virtual bool isSwitchByEventInStateResumeLowBatteryCharge();
	virtual bool updateActionInStateResumeLowBatteryCharge(){};
	virtual void switchInStateResumeLowBatteryCharge(){};

	// State charge
	bool isStateCharge() const
	{
		return sp_state == state_charge.get();
	}
	virtual bool isSwitchByEventInStateCharge(){return false;};
	virtual bool updateActionStateCharge(){};
	virtual void switchInStateCharge(){};

	// State pause
	bool isStatePause() const
	{
		return sp_state == state_pause.get();
	}
	virtual bool isSwitchByEventInStatePause(){return false;};
	virtual bool updateActionInStatePause(){};
	virtual void switchInStatePause(){};

	// State desk test
	bool isStateDeskTest() const
	{
		return sp_state == state_test.get();
	}
	virtual bool isSwitchByEventInStateDeskTest(){return false;};
	virtual bool updateActionInStateDeskTest(){return false;};
	virtual void switchInStateDeskTest(){};

	bool is_closed{true};
	bool is_isolate{true};
	int closed_count_{};
	int closed_count_limit_{2};
	int isolate_count_{};
	int isolate_count_limit_{3};
	bool is_trapped_{false};

	boost::shared_ptr<State> state_init{new StateInit()};
	boost::shared_ptr<State> state_clean{new StateClean()};
	boost::shared_ptr<State> state_exception_resume{new StateExceptionResume()};
	boost::shared_ptr<State> state_exploration{new StateExploration()};

	Points passed_path_{};
	typedef std::set<PairCell_t> Blocks_t ;
	Blocks_t c_blocks;
	Points plan_path_{};
	bool found_temp_charger_{};
	bool in_rcon_signal_range_{};
	bool should_mark_charger_{};
	bool should_follow_wall{};
	bool should_mark_temp_charger_{};

	Dir_t old_dir_{};
	Point_t start_point_{};
	DequeArray<Point_t> history_{};
	Point_t iterate_point_{};

//	boost::shared_ptr<APathAlgorithm> follow_wall_path_algorithm_{};
	boost::shared_ptr<APathAlgorithm> clean_path_algorithm_{};
	boost::shared_ptr<GoHomePathAlgorithm> go_home_path_algorithm_{};
	GridMap clean_map_{};
	static bool plan_activation_;
	double time_gyro_dynamic_;

protected:
	std::vector<State*> sp_saved_states;
	boost::shared_ptr<State> state_go_home_point{new StateGoHomePoint()};
	boost::shared_ptr<State> state_go_to_charger{new StateGoToCharger()};
	boost::shared_ptr<State> state_charge{new StateCharge()};
	boost::shared_ptr<State> state_folllow_wall{new StateFolllowWall()};
	boost::shared_ptr<State> state_spot{new StateSpot()};
	boost::shared_ptr<State> state_resume_low_battery_charge{new StateResumeLowBatteryCharge()};
	boost::shared_ptr<State> state_pause{new StatePause()};
	boost::shared_ptr<State> state_test{new StateTest()};

	bool low_battery_charge_{};
	bool moved_during_pause_{};
	Points home_points_{};
	bool should_go_to_charger_{false};
	bool remote_go_home_point{false};
	bool first_time_go_home_point_{true};
	bool seen_charger_during_cleaning_{false};
	bool go_home_for_low_battery_{false};
	bool switch_is_off_{false};
	Points charger_pose_;
	Points tmp_charger_pose_;
	bool found_charger_{false};
	bool out_range_charger_{false};
public:

	static void pubPointMarkers(const std::deque<Vector2<double>> *point, std::string frame_id,std::string name);
	void pubFitLineMarker(visualization_msgs::Marker fit_line_marker);
	void setLinearCleaned();
	uint8_t setFollowWall(GridMap&, bool is_left, const Points&);
	void scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr& scan);
	void setCleanMapMarkers(int16_t x, int16_t y, CellState type,  visualization_msgs::Marker& clean_map_markers_);
	void pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path);
	static void pubLineMarker(const std::vector<LineABC> *lines);
	Vector2<double> getTargetPoint(const Vector2<double> &p1, const Vector2<double> &p2, const PointSelector &para);
	bool removeCrossingPoint(const Vector2<double> &target_point, PointSelector &para,
													 const sensor_msgs::LaserScan::ConstPtr &scan);
	bool checkCorner(const sensor_msgs::LaserScan::ConstPtr &scan, const PointSelector &para);
	bool calcLidarPath(const sensor_msgs::LaserScan::ConstPtr & scan,bool is_left ,std::deque<Vector2<double>>& points, double wall_distance);
//	Vector2<double> polarToCartesian(double polar, int i);
	void setTempTarget(std::deque<Vector2<double>>& points, uint32_t  seq);
	void pubTmpTarget(const Point_t &point,bool is_virtual=false);
	uint8_t setBlocks(Dir_t dir);
	void saveBlocks();
	void saveBlock(int block, int , std::function<Cells()>);
	void checkShouldMarkCharger(float angle_offset,float distance);
	PathHead getTempTarget();

private:
	PathHead path_head_{};
	visualization_msgs::Marker bumper_markers_;
	ros::NodeHandle clean_nh_;
	ros::Subscriber map_sub_;
	ros::Subscriber	scanLinear_sub_;
	ros::Subscriber	scanCompensate_sub_;
	ros::Subscriber lidarPoint_sub_;
	ros::Subscriber	scanOriginal_sub_;

	ros::Publisher tmp_target_pub_;
	static ros::Publisher point_marker_pub_;
	ros::Publisher send_clean_map_marker_pub_;
	ros::Publisher line_marker_pub_;
	static ros::Publisher line_marker_pub2_;
	ros::Publisher fit_line_marker_pub_;
	boost::mutex temp_target_mutex_;


};

class CleanModeNav:public ACleanMode
{
public:
	CleanModeNav();
	~CleanModeNav();

	bool mapMark() override;
	bool markRealTime() override;
	bool isExit() override;

	bool allowRemoteUpdatePlan() override
	{
		return isStatePause();
	}

	void keyClean(bool state_now, bool state_last) override ;
	void remoteHome(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
	void remoteDirectionLeft(bool state_now, bool state_last) override ;
	void remoteDirectionRight(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;
	void remoteWallFollow(bool state_now, bool state_last) override ;
	void remoteSpot(bool state_now, bool state_last) override;
	void remoteMax(bool state_now, bool state_last) override;
	void remotePlan(bool state_now, bool state_last) override;
//	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
	void batteryHome(bool state_now, bool state_last) override ;
//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;

//	void overCurrentVacuum(bool state_now, bool state_last);

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

	// State spot
	bool checkEnterTempSpotState();
	bool isSwitchByEventInStateSpot() override;
	void switchInStateSpot() override;

	// State pause
	bool checkEnterPause();
	bool checkResumePause();
	bool isSwitchByEventInStatePause() override;
	bool updateActionInStatePause() override;

	// State folllow wall
	bool isSwitchByEventInStateFollowWall() override;

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
	bool has_aligned_and_open_slam_{};
	float paused_odom_radian_{};
	float start_align_radian_{};
//	bool is_stay_in_same_postion_long_time{};
	Point_t continue_point_{};
	static int align_count_;
};

class CleanModeExploration : public ACleanMode
{
public:
	CleanModeExploration();
	~CleanModeExploration();

	bool markMapInNewCell() override;
	bool mapMark() override;
	void resetErrorMarker();
//	bool isExit() override;
	void keyClean(bool state_now, bool state_last) override ;
	void remoteClean(bool state_now, bool state_last) override ;
//	void cliffAll(bool state_now, bool state_last) override ;
	void chargeDetect(bool state_now, bool state_last) override ;
	void remoteMax(bool state_now, bool state_last) override ;

//	void overCurrentBrushLeft(bool state_now, bool state_last);
//	void overCurrentBrushMain(bool state_now, bool state_last);
//	void overCurrentBrushRight(bool state_now, bool state_last);
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
//	void overCurrentVacuum(bool state_now, bool state_last);
//	void printMapAndPath();
	void switchInStateInit() override;
	bool updateActionInStateInit() override;

	void switchInStateGoHomePoint() override;
	void switchInStateGoToCharger() override;

//	bool moveTypeFollowWallIsFinish(MoveTypeFollowWall *p_mt) override;

private:
	bool mark_robot_{true};
	Marks error_marker_;
};

class CleanModeFollowWall:public ACleanMode {
public:
	CleanModeFollowWall();

	~CleanModeFollowWall() override;

	bool isExit() override;
	bool mapMark() override;

	void keyClean(bool state_now, bool state_last) override;
	void remoteMax(bool state_now, bool state_last) override;
	void remoteClean(bool state_now, bool state_last) override;
	void remoteWallFollow(bool state_now, bool state_last) override;
	void chargeDetect(bool state_now, bool state_last) override;
	void switchInStateFollowWall() override;

	void switchInStateInit() override;

private:
 int reach_cleaned_count_save{};
};

class CleanModeSpot:public ACleanMode
{
public:
	CleanModeSpot();
	~CleanModeSpot();

	bool mapMark() override;
	bool isExit() override;
//	void cliffAll(bool state_now, bool state_last) override;
	void remoteClean(bool state_now, bool state_last) override;
	void keyClean(bool state_now, bool state_last) override;
	void remoteSpot(bool state_now,bool state_last) override ;
	void switchInStateInit() override ;
	void switchInStateSpot() override ;
	void overCurrentWheelLeft(bool state_now, bool state_last) override;
	void overCurrentWheelRight(bool state_now, bool state_last) override;
private:

};

class CleanModeTest:public ACleanMode
{
public:
	CleanModeTest(uint8_t mode);
	~CleanModeTest();

	bool mapMark() override
	{return false;}

	bool isFinish() override;
	bool isExit() override;
	bool updateActionInStateDeskTest() override;
	void switchInStateDeskTest() override;

	void keyClean(bool state_now, bool state_last) override ;
	void remoteDirectionForward(bool state_now, bool state_last) override ;

private:
	uint8_t test_mode_{0};
};
#endif //PP_MODE_H_H
