//
// Created by austin on 18-1-26.
//

#ifndef PP_R16_BOARD_TEST_HPP
#define PP_R16_BOARD_TEST_HPP

#endif //PP_R16_BOARD_TEST_HPP

#include "config.h"

#if R16_BOARD_TEST

#include "dev.h"
#include "robot.hpp"
#include "robotbase.h"
/*
 * Board test for r16.(Not completed)
 *
 * Current containing test items:
 * 1. Speaker.
 * 2. Serial port.
 * 3. USB device.
 * 4. Power supply.
 *
 */
void r16_board_test(std::string serial_port, int baud_rate);

/*
 * Dead loop for error.
 */
void error_loop();

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
#endif
