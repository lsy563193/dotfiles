//
// Created by root on 11/17/17.
//

#ifndef PP_VACUUM_H
#define PP_VACUUM_H

#include <cstdint>

enum VacSpeed{
 Vac_Speed_Max	=			100, //15500rpm
 Vac_Speed_Normal	=		80, //12000rpm
 Vac_Speed_Low	=	50 ,//8000rpm
};

#define TWO_HOURS					7200

class Vacuum {
public:

/*
 * Set the mode for vacuum.
 * The mode should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_Low/Vac_Save
 * para
 * mode: Vac_Normal Vac_Max Vac_Save(load mode save last time)
 * save: if save is ture,save this mode,next time clean will reload at interface
 * */

	void setTmpSpotState();

	void setTmpLowState();

	bool isMaxInClean() //getter
	{
		return is_max_clean_state_;//getter
	}

	void isMaxInClean(bool is_max)
	{
		is_max_clean_state_ = is_max;
	}

	void setCleanState();

	void stop();

	void bldcSpeed(uint32_t S);

	void startExceptionResume(void);

	void resetExceptionResume(void);

	void setOc(bool val)
	{
		oc_ = val;
	}
	bool getOc() const {
		return oc_;
	}

	void setCurrent(uint16_t current)
	{
		current_ = current;
	}

	uint16_t getCurrent()
	{
		return current_;
	}

private:
	bool is_max_clean_state_{};
	bool oc_;

	uint16_t current_;
};

extern Vacuum vacuum;

#endif //PP_VACUUM_H
