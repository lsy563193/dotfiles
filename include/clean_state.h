//
// Created by lsy563193 on 17-9-27.
//

#ifndef PP_CLEAN_STATE_H
#define PP_CLEAN_STATE_H

enum{
	CS_OPEN_GYRO,
	CS_BACK_FROM_CHARGER,
	CS_OPEN_LIDAR,
	CS_ALIGN,
	CS_OPEN_SLAM,
	CS_CLEAN,
	CS_GO_HOME_POINT,
	CS_GO_CHANGER,
	CS_TMP_SPOT,
	CS_TRAPPED,
	CS_EXPLORATION,
	CS_SELF_CHECK,
	CS_STOP,
	CS_NUM,
};

class CleanStateBase {
public:
	CleanStateBase() {
		cs_ = 0;
	}

public:
	bool init();

	bool is_open_gyro();

	bool is_back_from_charger();

	bool is_open_lidar();

	bool is_align();

	bool is_open_slam();

	bool is_going_home();

	bool is_go_home_point();

	bool is_go_charger();

	bool is_exploration();

	bool is_clean();

	bool is_tmp_spot();

	bool is_trapped();

	bool is_self_check();

	int get(void);

	void setNext(int);

	virtual bool cs_next(const Cell_t& start, PPTargetType& path)=0;
	virtual void setting()=0;

protected:
	bool isTrapped();

protected:
	static int cs_;//clean state
};

class TrappedCS:public CleanStateBase
{
public:
	bool cs_next(const Cell_t& start, PPTargetType& path);
	void setting();
};

class CleanCS:public CleanStateBase
{
public:
	bool cs_next(const Cell_t& start, PPTargetType& path);
	void setting();
};

class TmpSpotCS:public CleanStateBase
{
public:
	bool cs_next(const Cell_t& start, PPTargetType& path);
	void setting();
};

class GoHomePointCS:public CleanStateBase
{
public:
	void setting(void);
	bool cs_next(const Cell_t& start, PPTargetType& path);
};

class ExplorationCS:public CleanStateBase
{
public:
	bool cs_next(const Cell_t& start, PPTargetType& path);
	void setting(void);
};

class SelfCheckCS:public CleanStateBase
{
public:
	void setting(void);
	bool cs_next(const Cell_t& start, PPTargetType& path);
};

class GoChargeCS:public CleanStateBase
{
public:
	void setting(void);
	bool cs_next(const Cell_t& start, PPTargetType& path);
};

class CleanStateManage:public CleanStateBase
{

public:
	CleanStateManage();
	void setting(void);
	bool cs_next(const Cell_t& start, PPTargetType& path);

private:
	std::array<CleanStateBase*,CS_NUM> vss_;
};

extern CleanStateManage cs;
//extern CleanStateBase* p_cs;

#endif //PP_CLEAN_MODE_H
