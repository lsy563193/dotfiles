//
// Created by root on 11/17/17.
//

#ifndef PP_VACUUM_H
#define PP_VACUUM_H

#define Vac_Speed_Max				100 //15500rpm
#define Vac_Speed_Normal			60 //9000rpm
#define Vac_Speed_NormalL			50 //8000rpm

enum {
Vac_Normal=0,
Vac_Max,
};

class Vacuum {
public:
	Vacuum();

/*
 * Set the mode for vacuum.
 * The mode should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_NormalL/Vac_Save
 * para
 * mode: Vac_Normal Vac_Max Vac_Save(load mode save last time)
 * save: if save is ture,save this mode,next time clean will reload at interface
 * */

	void setTmpMode(uint8_t mode);

	void setMode(uint8_t mode);

	void setLastMode();

	void stop();

	uint8_t getMode(void)
	{
		return mode_;
	}

	void bldcSpeed(uint32_t S);

	void startSelfCheck(void);

	void resetSelfCheck(void);

	void setSelfCheckStatus(uint8_t val)
	{
		self_check_status_ = val;
	}
	uint8_t getSelfCheckStatus() const {
		return self_check_status_;
	}

	void setOc(bool val)
	{
		oc_ = val;
	}
	bool getOc() const {
		return oc_;
	}

private:
	void setSpeedByMode(uint8_t);

	uint8_t mode_;
	uint8_t mode_save_;
	uint8_t self_check_status_;
	bool oc_;

};

extern Vacuum vacuum;

#endif //PP_VACUUM_H
