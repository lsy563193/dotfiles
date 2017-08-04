#ifndef __CONFIG_H__
#define __CONFIG_H__
/*robot type define,currently got two type "ROBOT_X900" "ROBOT_X400"*/
#define __ROBOT_X900 							(1)
#define __ROBOT_X400							(0)
/* ------------------------------------- System Setting ------------------------------------- */

/*
 * Chipset setting.
 */
#define CPU_CHIP			(91)

/*
 * Build system, CPU ID checking will be different for building on Linux or Window.
 */
#define BUILD_ON_LINUX			(1)

/*
 * Gyro dynamic setting. If on, it will enable gyro dynamic adjustment while robot turning.
 */
#define GYRO_DYNAMIC_ADJUSTMENT	(1)

/* ------------------------------------- Path Planning ------------------------------------- */

/*
 * Definition for enable debug or not.
 */
#define ENABLE_DEBUG			(1)

/*
 * Enable debugging the grid map.
 */
#define DEBUG_MAP			(1)

/*
 * Enable debugging the shortest path map when finding the
 * shortest path is using the A* like method.
 */
#define DEBUG_SM_MAP			(1)

/*
 * Enable target debug.
 */
#define DEBUG_TARGETS			(1)

/*
 * Definition to enable/disable robot rounding the obstcal or not.
 */
#define PP_ROUNDING_OBSTACLE_LEFT	(1)
#define PP_ROUNDING_OBSTACLE_RIGHT	(1)
/*
 * Definition to enable/disable robot curve move support.
 */
#define PP_CURVE_MOVE			(1)

/*
 * Definition to force robot to move the center of a cell.
 *
 * If not defined, when moving to a cell, it will just use the
 * curent positions of X or Y encoder count only.
 */
#define PP_MOVE_TO_CELL_CENTER		(1)

/*
 * Definition to try to modify the path obtained from A-start to a target,
 * so that the robot can better avoid the obstacles.
 */
#define PP_MOVE_TO_MIDDLE_OF_PATH	(1)

/* Zig-Zag cleanning time 120 minutes */
#define CLEANNING_TIME			(7200)

/* With water tank, Zig-Zag cleanning time 90 minutes */
#define WET_CLEANNING_TIME		(5400)

/* Wall follow cleanning time 60 minutes */
#define WALL_FOLLOW_TIME		(3600)

/* Low battery go home voltage value */
#define LOW_BATTERY_GO_HOME_VOLTAGE		(1320)

/* Low battery stop robot voltage value */
#define LOW_BATTERY_STOP_VOLTAGE		(1250)

/* Ready to clean battery voltage value */
#define BATTERY_READY_TO_CLEAN_VOLTAGE	(1400)

/* Fully charged battery voltage value */
#define BATTERY_FULL_VOLTAGE	(1600)

/* OBS setting */
#define OBS_DYNAMIC			(1)
#define OBS_DYNAMIC_MOVETOTARGET (1)
#define WALL_DYNAMIC    (1)

/*
 * If defined, enable virtual wall.
 */
#define VIRTUAL_WALL			(0)

/*
 * Defines for maximum distance between 2 virtual wall points,
 * If less than this value, points between these two points will be
 * marked as unaccessible.
 */
#define VIRTUAL_WALL_MAX_SEGMENT_DISTANCE	(6)

/* Bumper and Cliff Error */
#define CLIFF_ERROR    (1)
#define BUMPER_ERROR   (1)

/*
 * Definition relates to the robot.
 */

/*
 * If ROBOT_SIZE equals to 5, robot is defined as occupying 25(5x5) cells.
 * If it equals to 3, robot is defined as occupying 9(3x3) cells
 */
#define ROBOT_SIZE			(3)

#if (ROBOT_SIZE == 5)

#define ROBOT_HEAD_OFFSET		(3)
#define ROBOT_LEFT_OFFSET		(2)
#define ROBOT_RIGHT_OFFSET		(-2)

#define ROBOT_BRUSH_LENGTH		(3)
#define ROBOT_BRUSH_LEFT_OFFSET		(-1)
#define ROBOT_BRUSH_RIGHT_OFFSET	(1)

#define SLOW_DOWN_DISTANCE		(2 * CELL_COUNT_MUL)

#else

#define ROBOT_HEAD_OFFSET		(2)
#define ROBOT_LEFT_OFFSET		(1)
#define ROBOT_RIGHT_OFFSET		(-1)

#define ROBOT_BRUSH_LENGTH		(1)
#define ROBOT_BRUSH_LEFT_OFFSET		(0)
#define ROBOT_BRUSH_RIGHT_OFFSET	(0)

#define SLOW_DOWN_DISTANCE		(CELL_COUNT_MUL)

#endif

#define ROBOT_SIZE_1_2			(ROBOT_SIZE/2)
/*
 * Total number of targets that can be allowed..
 */
#define TARGET_TOTAL_COUNT		(MAP_SIZE * 5 / 2)

/* ------------------------------------- Shortest Path ------------------------------------- */

/*
 * Enable debugging the shortest path when using line-segments
 * strategy.
 */
#define DEBUG_POS			(1)

/*
 * Defines for when strategy to be used for shortest path. If
 * defines SHORTEST_PATH_V2, line-segment strategy will be use,
 * otherwise, it will use A* like strategy.
 */
//#define SHORTEST_PATH_V2		(1)

/*
 * When defines SHORTEST_PATH_V2_RAM, line-segment & targets will
 * be saved by using dymanic memory allocation, but that is not
 * statble currently.
 */
//#define SHORTEST_PATH_V2_RAM		(1)

/*
 * Total number of lines that the shorestpath will search for.
 */
#define POS_LINE_CNT			(780)

/* ------------------------------------- Path Planning Wall Follow ------------------------------------- */

/*
 * If defined DISABLE_WALL_ALIGN, robot will disable wall
 * alignment when it starts, otherwise, it when it starts,
 * it will move for a short distance and align the Gyro angle
 * with the wall.
 */
#define DISABLE_WALL_ALIGN		(1)

/*
 * If defined DISABLE_WALL_ALIGN_SPOT_MODE, remote spot key even won't be handled.
 */
#define DISABLE_WALL_ALIGN_SPOT_MODE	(1)

/*
 * Define find wall method
 */
//#define FIND_WALL_ANGLE_METHOD_1		(1)
#define FIND_WALL_ANGLE_METHOD_2		(1)

/* 5 meters for finding wall, align the starting angle for Gyro */
#define MAP_FIND_WALL_DISTANCE		(67 * 4)

/* Escape time set to 9 minutes 540s*/
#define ESCAPE_TRAPPED_TIME		(540)

/* Set trapped reference target size for robot to check that if it is trapped */
#define ESCAPE_TRAPPED_REF_CELL_SIZE		(3)

/* Set maximum bumper count of complicated area */
#define COMPLICATED_AREA_BUMPER_MAX_COUNT		(10)

/* ------------------------------------- Path Planning Gyro ------------------------------------- */

/*
 * Define which Gyro will be use.
 * If defined GYRO_XV7011, it will use XV70XX from Espon.
 * If not defined, it will use RN13XX from MicroInfinity
 */
#define GYRO_XV7011			(1)

/*
 * Define for enabling/disabling Gyro realtime calibration.
 * Default is disabled.
 */
#define GYRO_CALIBRATION             (1)

/*
 * Defines for enabling/disabling tilted detect or not.
 * When detecting the robot is tilted, it will check the all the 3 cliff
 * sensors value, which if all are less then 1500, and at the same time,
 * if the gyro X-axis & Y-axis angle are greater than 5 degree, we will
 * confirmed that the robot is tilted.
 */
//#define ENABLE_TILTED_DETECT		(1)

/*
 * Value for maximum allowed angle while the robot is tilted. If it is greater
 * than this angle, the robot is tilted.
 */
#define TILTED_ANGLE_LIMIT		(75)

/*
 * Value for lower limit of all the 3 cliff sensors, if all those 3 sensors have
 * values lower than this value, the robot is tilted.
 */
#define TILTED_CLIFF_LIMIT		(1500)

/* ------------------------------------- Core Move ------------------------------------- */

/*
 * Distance between left & right wheel in mm.
 */
#define WHEEL_BASE			(218)

/*
 * Range of LINEAR_MIN_SPEED should be better within 12 to 15.
 * When it is too small, it will move like shaking when robot startup.
 * When it is too large, it will fall down when reach the cliff.
 */
#define LINEAR_MIN_SPEED						((int32_t) 15) // 15)
#define LINEAR_MAX_SPEED						((int32_t) 40) // 15)
#define ROTATE_TOP_SPEED				((uint8_t) 22) // 22)
#define ROTATE_LOW_SPEED				((uint8_t) 7)
#if __ROBOT_X900
#define RUN_TOP_SPEED					((int32_t) 40) // 45)
#elif __ROBOT_X400

#define RUN_TOP_SPEED					((int32_t) 43) // 45)
#endif
/*
 * Go home using CM_MoveToCell function
 */
#define GO_HOME_METHOD_2		(1)

/*
 * How to add targets in zone, if defined, will add targets by using wall
 * follow path, otherwise, will just add radomly.
 */
#define ADD_TARGET_BY_PATH		(1)
#define ADD_TARGET_BY_PATH_ONLY		(1)


// Config for whether enable the function of continuing cleanning after charge
#define CONTINUE_CLEANING_AFTER_CHARGE	(1)

//#if CONTINUE_CLEANING_AFTER_CHARGE
/* Continue cleaning voltage value */
#define CONTINUE_CLEANING_VOLTAGE	(1530)
//#endif

// Config for whether enable the function of manual pause cleaning.
#define MANUAL_PAUSE_CLEANING	(1)

/* ------------------------------------- Path Planning Map ------------------------------------- */

/*
 * Definition of the grid map.
 */
#define MAP_DIMENSION			(200 * CELL_SIZE)//CELL_SIZE * 190 //9225//7925  // 14250
#define MAP_SIZE			(MAP_DIMENSION / CELL_SIZE)

/*
 * Definition relates to a grid cell.
 */
#if (ROBOT_SIZE == 5)
#define CELL_SIZE			(65) // 65
#define CELL_SIZE_2			(2 * CELL_SIZE) // 65
#define CELL_SIZE_3			(3 * CELL_SIZE) // 65
#define CELL_COUNT_MUL			(349) // 375  // 207
#define CELL_COUNT_MUL_1_2		(175) // 187  // 103

#else

#if __ROBOT_X900
#define SPEED_ALF    (7.83)
#define CELL_SIZE			(112) // 65
#define CELL_SIZE_2			(2 * CELL_SIZE) // 65
#define CELL_SIZE_3			(3 * CELL_SIZE) // 65
#define CELL_COUNT_MUL			(573) // 375  // 207
#define CELL_COUNT_MUL_1_2		(286) // 187  // 103
#elif __ROBOT_X400
#define SPEED_ALF    (7.23)
#define CELL_SIZE			(103) // 65
#define CELL_SIZE_2			(2 * CELL_SIZE) // 65
#define CELL_SIZE_3			(3 * CELL_SIZE) // 65
#define CELL_COUNT_MUL			(553) // 375  // 207
#define CELL_COUNT_MUL_1_2		(276) // 187  // 103
#endif
#endif

/* ------------------------------------- Zone Wall Follow ------------------------------------- */

/*
 * Enable or disable zone cleaning mode.
 */
#define ZONE_WALLFOLLOW		(1)

/*
 * Enable or disable escape mode
 */
//#define ZONE_ESCAPE		(1)

/*
 * Zone definitions.
 */

#define ZONE_DIMENSION			(CELL_SIZE * ZONE_SIZE)//3090  //1430  // 910  // 1950
#define ZONE_SIZE				(30)
#define ZONE_SIZE_HALF			(ZONE_SIZE / 2)

/*
 * Zone size of keeping obstacals
 */
#define ZONE_KEEP_OBSTACALS_SIZE	(2)

/*
 * Number of maximum zone allowed.
 */
#define ZONE_MAX			((MAP_DIMENSION * MAP_DIMENSION / ZONE_DIMENSION / ZONE_DIMENSION) * 4)

/*
 * Maximum number of zone allowed, if greater than this count, force go home.
 */
#define FROCE_GO_HOME_ZONE_CNT		(25)

/*
 * Boundary increment, extend the boundary with this value
 */
#define BOUNDARY_INCREMENT			(3)

/* ----------------- Defines about how to stop when cleaning zone mode. ----------------- */
/*
 * Stop Robot clean method 1.
 */
#define STOP_WALL_FOLLOW_M1			(1)

/*
 * Maximum distance for method 1 which will cause the robot to stop cleaning.
 * The value is the distance between start point of current zone and start point of
 * zone 1, 2, or 3. Also is for the end point to end point of the zones for matching.
 *
 * This value is calculated in cell size.
 */
#define STOP_WALL_FOLLOW_M1_MAX_DISTANCE	(7)

//#define STOP_WALL_FOLLOW_M2			(1)
#define STOP_WALL_FOLLOW_M3			(1)
/*
 * Check zone size of method 1
 */
#define STOP_WALL_FOLLOW_M1_CHECK_ZONE_SIZE		(3)
#define STOP_WALL_FOLLOW_M3_CHECK_ZONE_SIZE		(3)
/*
 * Define that if create middle zone to clean after finish normal wall follow cleaning
 */
//#define MIDDLE_ZONE				(1)

/* ------------------------------------- Alignment ------------------------------------- */
/*
 * Enable alignment
 */
#define ALIGNMENT_ENABLE			(1)

/*
 * Define that the alignment will use the wall angle to directly alignment gyro angle
 */
#define ALIGNMENT_ANGLE_WALL_ENABLE		(1)

/*
 *Define that alignement will use the gyro data instead of line angle to alignment gyro angle
 */
//#define OFFSET_ANGLE_GYRO_DATA			(1)
#endif

#define LIMIT_DISTANCE_ENABLE				(0)
#define EXPLORE_SCOPE_ENABLE				(0)

/* Define the display mode in user interface routine*/
#define ONE_KEY_DISPLAY                     (1)

/* Timeout seconds setting in user interface routine*/
#define USER_INTERFACE_TIMEOUT				(600)

#define STANDARD_REMOTE   (1)

/* Define for checking CPU ID & key verification with robot's MCU. */
#define	VERIFY_DEBUG						(0)
#define VERIFY_CPU_ID						(0)
#define VERIFY_KEY							(0)

/* ------------------------------------- Slam method for testing ------------------------------------- */
#define SLAM_METHOD_2						(1)

/* ------------------------------------- Laser Follow Wall Enable ------------------------------------- */
#define LASER_FOLLOW_WALL					(1)

/* ------------------------------------- Laser Marker Enable ------------------------------------- */
#define LASER_MARKER					(1)
#define FORCE_MOVE_LINE					(1)
