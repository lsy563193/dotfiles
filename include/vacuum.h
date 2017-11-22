//
// Created by root on 11/17/17.
//

#ifndef PP_VACUUM_H
#define PP_VACUUM_H

#define Vac_Speed_Max				100 //15500rpm
#define Vac_Speed_Normal			60 //9000rpm
#define Vac_Speed_NormalL			50 //8000rpm

#include <pp/x900sensor.h>
#include "controller.h"
extern pp::x900sensor sensor;
enum {
Vac_Normal=0,
Vac_Max,
Vac_Save,
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

	void mode(uint8_t mode, bool is_save);

	void mode(uint8_t mode);

	void stop();

	void switchToNext(bool is_save);

	uint8_t mode(void);

	void bldc_speed(uint32_t S);

	void start_self_check(void) {
		uint8_t omni_reset_byte = controller.get(CTL_OMNI_RESET);
		controller.set(CTL_OMNI_RESET, omni_reset_byte | 0x02);
	}

	void reset_self_check(void) {
		uint8_t omni_reset_byte = controller.get(CTL_OMNI_RESET);
		controller.set(CTL_OMNI_RESET, omni_reset_byte & ~0x06);
	}

	uint8_t get_self_check_status(void) {
		return (uint8_t) getVacuumSelfCheckStatus();
	}

	uint8_t getVacuumSelfCheckStatus() const {
		return sensor.vacuum_selfcheck_status;
	}

	bool getVacuumOc() const {
		return sensor.vcum_oc;
	}

private:
	void set_speed_by_mode(void);

	uint8_t mode_;
	uint8_t mode_save_;

};

extern Vacuum vacuum;

#endif //PP_VACUUM_H
