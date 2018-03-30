//
// Created by root on 11/17/17.
//

#ifndef PP_VACUUM_H
#define PP_VACUUM_H

#include <cstdint>

#define Vac_Speed_Max				100 //15500rpm
#define Vac_Speed_Normal			80 //12000rpm
#define Vac_Speed_Low			50 //8000rpm

#define TWO_HOURS					7200

enum {
	Vac_Normal=0,
	Vac_Max=1,
};

class Vacuum {
public:
	Vacuum();

/*
 * Set the mode for vacuum.
 * The mode should be Vac_Speed_Max/Vac_Speed_Normal/Vac_Speed_Low/Vac_Save
 * para
 * mode: Vac_Normal Vac_Max Vac_Save(load mode save last time)
 * save: if save is ture,save this mode,next time clean will reload at interface
 * */

	void setTmpMode(uint8_t mode);

	void setMode(uint8_t mode);

	void Switch();

	void setLastMode();

	void stop();

	uint8_t getMode(void)
	{
		return mode_;
	}

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
	void setSpeedByMode(uint8_t);

	uint8_t mode_ = Vac_Normal;
	uint8_t mode_save_;
	bool oc_;

	uint16_t current_;
};

extern Vacuum vacuum;

#endif //PP_VACUUM_H
