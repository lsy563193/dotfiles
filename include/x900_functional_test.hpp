//
// Created by austin on 18-1-26.
//

#ifndef PP_X900_FUNCTIONAL_TEST_HPP
#define PP_X900_FUNCTIONAL_TEST_HPP

#include "config.h"
#include "error.h"

//limit
#define OBS_MANUAL_LIMIT_H	(uint16_t)1500
#define OBS_MANUAL_LIMIT_L	(uint16_t)700
#define WALL_MANUAL_LIMIT_H	(uint16_t)500
#define WALL_MANUAL_LIMIT_L	(uint16_t)140
#define OBS_FIXTURE_LIMIT_H	(uint16_t)1500
#define OBS_FIXTURE_LIMIT_L	(uint16_t)700
#define WALL_FIXTURE_LIMIT_H	(uint16_t)500
#define WALL_FIXTURE_LIMIT_L	(uint16_t)140

//index for array "baseline"
#define LEFT_WHEEL			0
#define RIGHT_WHEEL			1
#define LEFT_BRUSH			2
#define MAIN_BRUSH			3
#define RIGHT_BRUSH			4
#define VACUUM					5
#define REF_VOLTAGE_ADC	6
#define SYSTEM_CURRENT	7

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
void error_loop(uint16_t error_code);

/*
 * Test RAM.
 */
bool RAM_test();

/*
 * Test flash.
 */
bool Flash_test();

/*
 * Test serial port.
 */
bool serial_port_test();

/*
 * Test lidar.
 */
bool lidar_test();

/*
 * Test lidar bumper.
 */
bool lidar_bumper_test();

/*
 * Test for power supply voltage.
 */
bool power_supply_test();

/*
 * Test for hardware on main board.
 */
uint16_t main_board_test();

uint16_t electrical_specification_and_led_test(uint16_t* baseline, bool &is_fixture, uint8_t &test_stage);

uint16_t cliff_test(uint8_t &test_stage);
uint16_t bumper_test(uint8_t &test_stage);
uint16_t obs_test(uint8_t &test_stage, bool is_fixture);
uint16_t rcon_test(uint8_t &test_stage);
uint16_t wheels_test(uint8_t &test_stage, uint16_t *baseline);
uint16_t brushes_test(uint8_t &test_stage, uint16_t *baseline);
uint16_t charge_current_test(uint8_t &test_stage, bool is_fixture);
uint16_t vacuum_test(uint8_t &test_stage, uint16_t *baseline);
uint8_t get_charge_pwm_level(uint16_t voltage);
/*
 * Test for memory device.
 */
//bool memory_test();

#endif //PP_X900_FUNCTIONAL_TEST_HPP

