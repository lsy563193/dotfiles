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
#define MAIN_BOARD_ERROR 	((uint16_t)3006)

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
bool main_board_test();

/*
 * Test for memory device.
 */
//bool memory_test();
#endif
