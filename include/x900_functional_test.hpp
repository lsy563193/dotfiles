//
// Created by austin on 18-1-26.
//

#ifndef PP_X900_FUNCTIONAL_TEST_HPP
#define PP_X900_FUNCTIONAL_TEST_HPP

#endif //PP_X900_FUNCTIONAL_TEST_HPP

#include "config.h"

#if X900_FUNCTIONAL_TEST

// Definition for error code.
#define SERIAL_ERROR 		((uint16_t)3001)
#define RAM_ERROR 			((uint16_t)3002)
#define FLASH_ERROR 		((uint16_t)3003)
#define LIDAR_ERROR 		((uint16_t)3004)
#define LIDAR_BUMPER_ERROR 	((uint16_t)3005)
//#define MAIN_BOARD_ERROR 	((uint16_t)3006)
#define BASELINE_VOLTAGE_ERROR				(uint16_t)2101
#define BATTERY_ERROR						(uint16_t)2301
#define BATTERY_LOW							(uint16_t)2302
#define BASELINE_CURRENT_ERROR				(uint16_t)2401
#define BASELINE_CURRENT_LOW				(uint16_t)2402
#define LEFT_OBS_ERROR						(uint16_t)101
#define FRONT_OBS_ERROR						(uint16_t)102
#define RIGHT_OBS_ERROR						(uint16_t)103
#define LEFT_WALL_ERROR						(uint16_t)104
#define RIGHT_WALL_ERROR					(uint16_t)105
#define OBS_ENABLE_ERROR					(uint16_t)106
#define LEFT_BUMPER_ERROR					(uint16_t)201
#define RIGHT_BUMPER_ERROR					(uint16_t)202
#define LEFT_CLIFF_ERROR					(uint16_t)301
#define FRONT_CLIFF_ERROR					(uint16_t)302
#define RIGHT_CLIFF_ERROR					(uint16_t)303
#define LEFT_WHEEL_SW_ERROR					(uint16_t)304
#define RIGHT_WHEEL_SW_ERROR				(uint16_t)305
#define BLRCON_ERROR						(uint16_t)401
#define LRCON_ERROR							(uint16_t)402
#define FL2RCON_ERROR						(uint16_t)403
#define FLRCON_ERROR						(uint16_t)404
#define FRRCON_ERROR						(uint16_t)405
#define FR2RCON_ERROR						(uint16_t)406
#define RRCON_ERROR							(uint16_t)407
#define BRRCON_ERROR						(uint16_t)408
#define LEFT_WHEEL_FORWARD_CURRENT_ERROR	(uint16_t)501
#define LEFT_WHEEL_FORWARD_PWM_ERROR		(uint16_t)502
#define LEFT_WHEEL_FORWARD_ENCODER_FAIL		(uint16_t)503
#define LEFT_WHEEL_FORWARD_ENCODER_ERROR	(uint16_t)504
#define LEFT_WHEEL_BACKWARD_CURRENT_ERROR	(uint16_t)505
#define LEFT_WHEEL_BACKWARD_PWM_ERROR		(uint16_t)506
#define LEFT_WHEEL_BACKWARD_ENCODER_FAIL	(uint16_t)507
#define LEFT_WHEEL_BACKWARD_ENCODER_ERROR	(uint16_t)508
#define LEFT_WHEEL_STALL_ERROR				(uint16_t)509
#define RIGHT_WHEEL_FORWARD_CURRENT_ERROR	(uint16_t)601
#define RIGHT_WHEEL_FORWARD_PWM_ERROR		(uint16_t)602
#define RIGHT_WHEEL_FORWARD_ENCODER_FAIL	(uint16_t)603
#define RIGHT_WHEEL_FORWARD_ENCODER_ERROR	(uint16_t)604
#define RIGHT_WHEEL_BACKWARD_CURRENT_ERROR	(uint16_t)605
#define RIGHT_WHEEL_BACKWARD_PWM_ERROR		(uint16_t)606
#define RIGHT_WHEEL_BACKWARD_ENCODER_FAIL	(uint16_t)607
#define RIGHT_WHEEL_BACKWARD_ENCODER_ERROR	(uint16_t)608
#define RIGHT_WHEEL_STALL_ERROR				(uint16_t)609
#define	LEFT_BRUSH_CURRENT_ERROR			(uint16_t)701
#define LEFT_BRUSH_STALL_ERROR				(uint16_t)702
#define	RIGHT_BRUSH_CURRENT_ERROR			(uint16_t)801
#define RIGHT_BRUSH_STALL_ERROR				(uint16_t)802
#define	MAIN_BRUSH_CURRENT_ERROR			(uint16_t)901
#define MAIN_BRUSH_STALL_ERROR				(uint16_t)902
#define VACUUM_CURRENT_ERROR					(uint16_t)1001
#define VACUUM_PWM_ERROR							(uint16_t)1002
#define VACUUM_ENCODER_FAIL						(uint16_t)1003
#define VACUUM_ENCODER_ERROR					(uint16_t)1004
#define VACUUM_STALL_ERROR						(uint16_t)1005
//#define CHARGE_PWM_ERROR						(uint16_t)1101
//#define CHARGE_CURRENT_ERROR				(uint16_t)1102
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
uint16_t charge_current_test(uint8_t &test_stage);
uint16_t vacuum_test(uint8_t &test_stage, uint16_t *baseline);
uint8_t get_charge_pwm_level(uint16_t voltage);
/*
 * Test for memory device.
 */
//bool memory_test();
#endif
