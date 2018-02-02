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
 * 2. Serial port.
 * 3. USB device.
 * 4. Power supply.
 * 5. Flash and RAM.
 *
 */
void x900_functional_test(std::string serial_port, int baud_rate);

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

/*
 * Test for memory device.
 */
bool memory_test();
#endif
