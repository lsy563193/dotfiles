//
// Created by austin on 18-1-26.
//

#ifndef PP_X900_FUNCTIONAL_TEST_HPP
#define PP_X900_FUNCTIONAL_TEST_HPP

#include "config.h"
#include "error.h"

// ------------For functional test--------------
#define CTL_TESTING_STAGE 2
#define CTL_ERROR_CODE_HIGH 3
#define CTL_ERROR_CODE_LOW 4
// For motors test mode
// 0 for idle mode
// 1 for stall mode
#define CTL_LEFT_WHEEL_TEST_MODE 6
#define CTL_RIGHT_WHEEL_TEST_MODE 7
#define CTL_LEFT_BRUSH_TEST_MODE 2
#define CTL_MAIN_BRUSH_TEST_MODE 3
#define CTL_RIGHT_BRUSH_TEST_MODE 4
#define CTL_VACUUM_TEST_MODE 7
// For Charger Connected Status
// 0 for no charger connected
// 1 for already connect charger
#define CTL_CHARGER_CINNECTED_STATUS 3
// Is on fixture
#define CTL_IS_FIXTURE 2
// ------------For functional test end--------------

//limit
#define OBS_MANUAL_LIMIT_H	(uint16_t)2400
#define OBS_MANUAL_LIMIT_L	(uint16_t)1200
#define WALL_MANUAL_LIMIT_H	(uint16_t)500
#define WALL_MANUAL_LIMIT_L	(uint16_t)140
#define OBS_FIXTURE_LIMIT_H	(uint16_t)1500
#define OBS_FIXTURE_LIMIT_L	(uint16_t)700
#define WALL_FIXTURE_LIMIT_H	(uint16_t)500
#define WALL_FIXTURE_LIMIT_L	(uint16_t)140
#define SWING_CURRENT_LIMIT	(uint16_t)1000

//index for array "baseline"
#define LEFT_WHEEL			0
#define RIGHT_WHEEL			1
#define LEFT_BRUSH			2
#define MAIN_BRUSH			3
#define RIGHT_BRUSH			4
#define VACUUM					5
#define REF_VOLTAGE_ADC	6
#define SYSTEM_CURRENT	7
#define SWING_MOTOR			8

#include "dev.h"
#include "robot.hpp"
/*
 * Function for x900 machine.(Not completed)
 *
 * Current containing test items:
 * 1. Speaker.
 * 2. RAM.
 * 3. Flash.
 * 4. Serial port.
 * 5. WIFI module.
 * 6. Lidar.
 * 7. Lidar bumper.
 * (8. USB?)
 * 9. Main board hardware.
 *
 */
void x900_functional_test(std::string serial_port, int baud_rate, std::string lidar_bumper_dev);

/*
 * Dead loop for error.
 */
void error_loop(uint8_t test_stage, uint16_t error_code, uint16_t current_data);

/*
 * Test serial port.
 * return value:
 * 0: test pass
 * 1~254: main board version error, current main board version.
 * 255: serial error
 */
uint8_t serial_port_test();

/*
 * Test for power supply voltage.
 */
bool power_supply_test();

/*
 * Test for hardware on main board.
 */
void main_board_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);

void electrical_specification_and_led_test(uint16_t* baseline, bool &is_fixture, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);

void cliff_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void bumper_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void obs_test(bool is_fixture, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void rcon_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void water_tank_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void wheels_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void side_brushes_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void main_brush_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void charge_current_test(bool is_fixture, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data);
void vacuum_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_dara);
/*
 * Test for memory device.
 */
//bool memory_test();

#endif //PP_X900_FUNCTIONAL_TEST_HPP

