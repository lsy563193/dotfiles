//
// Created by austin on 18-1-26.
//

#ifndef PP_X900_FUNCTIONAL_TEST_HPP
#define PP_X900_FUNCTIONAL_TEST_HPP

#endif //PP_X900_FUNCTIONAL_TEST_HPP

#include "config.h"

#if X900_FUNCTIONAL_TEST

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
 * 5. Power supply.
 * 6. WIFI module.
 * 7. Lidar.
 * 8. Lidar bumper.
 * (9. USB?)
 * 10. Main board hardware.
 *
 */
void x900_functional_test(std::string serial_port, int baud_rate);

/*
 * Dead loop for error.
 */
void error_loop();

/*
 * Test RAM.
 */
bool RAM_test();

/*
 * Test flash.
 */
bool flash_test();

/*
 * Test serial port.
 */
bool serial_port_test();

/*
 * Test write on two usb drives (Used to test usb connection).
 */
bool usb_test(std::string dev_path, std::string fs_type, int write_length);

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
