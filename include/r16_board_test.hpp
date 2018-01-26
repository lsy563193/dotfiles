//
// Created by austin on 18-1-26.
//

#ifndef PP_R16_BOARD_TEST_HPP
#define PP_R16_BOARD_TEST_HPP

#endif //PP_R16_BOARD_TEST_HPP

#include "config.h"

#if R16_BOARD_TEST

#include "dev.h"
/*
 * Board test for r16.(Not completed)
 *
 * Current containing test items:
 * 1. Speaker.
 *
 */
void r16_board_test();

/*
 * Dead loop for error.
 */
void error_loop();
#endif
