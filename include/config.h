#ifndef __CONFIG_H__
#define __CONFIG_H__
/* ------------------------------------- System Setting ------------------------------------- */

/*
 * Chipset setting.
 */
#define CPU_CHIP			(91)

/*
 * Build system, CPU ID checking will be different for building on Linux or Window.
 */
#define BUILD_ON_LINUX			(1)

/* Define for checking CPU ID & key verification with robot's MCU. */
#define	VERIFY_DEBUG						(0)
#define VERIFY_CPU_ID						(0)
#define VERIFY_KEY							(0)

/* ------------------------------------- Speaker config ------------------------------------- */
#define SPEAKER_VOLUME						(55)

/* ------------------------------------- DEBUG config ------------------------------------- */
#define DEBUG_ENABLE						(1)
/* ------------------------------------- Grid Map config ------------------------------------- */

/*
 * Definition of the grid map.
 */
#define MAP_DIMENSION			(200 * CELL_SIZE)//CELL_SIZE * 190 //9225//7925  // 14250
//#define MAP_SIZE			(MAP_DIMENSION / CELL_SIZE)
#define MAP_SIZE			(300)
#define COLOR_DEBUG_MAP		(1)

/*
 * Definition relates to a grid cell.
 */
#define CELL_SIZE			(0.112f) // in meter
#define CELL_SIZE_2			(2 * CELL_SIZE) // 65
#define CELL_SIZE_3			(3 * CELL_SIZE) // 65
//#define CELL_COUNT_MUL			(573) // 375  // 207 // wheel encoder count
//#define CELL_COUNT_MUL_1_2		(286) // 187  // 103

/*
 * If ROBOT_SIZE equals to 5, robot is defined as occupying 25(5x5) cells.
 * If it equals to 3, robot is defined as occupying 9(3x3) cells
 */
#define ROBOT_SIZE			(3)

#define ROBOT_LEFT_OFFSET		(1)
#define ROBOT_RIGHT_OFFSET		(-1)

#define ROBOT_SIZE_1_2			(ROBOT_SIZE/2)

/*
 * Definition for enable debug or not.
 */
#define ENABLE_DEBUG			(1)

/*
 * Enable debugging the grid map.
 */
#define DEBUG_CLEAN_MAP			(1)

/*
 * Enable debugging the grid map.
 */
#define DEBUG_WF_MAP			(0)

/*
 * Enable debugging the shortest path map when finding the
 * shortest path is using the A* like method.
 */
#define DEBUG_COST_MAP			(1)

/*
 * Enable target debug.
 */
#define DEBUG_TARGETS			(1)

/* ------------------------------------- Timer ------------------------------------- */
/* Zig-Zag cleanning time 120 minutes */
#define CLEANNING_TIME			(7200)

/* Wall follow cleanning time 60 minutes */
#define WALL_FOLLOW_TIME		(3600)

/* Escape time set to 9 minutes 540s*/
#define ESCAPE_TRAPPED_TIME		(540)

/* Timeout seconds setting in user interface routine*/
#define IDLE_TIMEOUT				(600)

/* ------------------------------------- Battery config ------------------------------------- */
/* Low battery go home voltage value */
#define LOW_BATTERY_GO_HOME_VOLTAGE		(1350)

/* Low battery stop robot voltage value */
#define LOW_BATTERY_STOP_VOLTAGE		(1300)

/* Ready to clean battery voltage value */
#define BATTERY_READY_TO_CLEAN_VOLTAGE	(1400)

/* Fully charged battery voltage value */
#define BATTERY_FULL_VOLTAGE	(1640)

/* Continue cleaning voltage value */
#define RESUME_CLEANING_VOLTAGE	(1530)

/* Battery voltage criterion for brush setting*/
#define NORMAL_OPERATE_VOLTAGE_FOR_SIDE_BRUSH	(1050)
#define NORMAL_OPERATE_VOLTAGE_FOR_MAIN_BRUSH	(800)
#define SLOW_OPERATE_VOLTAGE_FOR_SIDE_BRUSH	(550)
#define SLOW_OPERATE_VOLTAGE_FOR_MAIN_BRUSH	(450)

/* Battery voltage criterion for water tank setting*/
#define FULL_OPERATE_VOLTAGE_FOR_SWING_MOTOR	(500)
#define LOW_OPERATE_VOLTAGE_FOR_SWING_MOTOR	(250)
/* ------------------------------------- Obs config ------------------------------------- */
/* OBS setting */
#define OBS_DYNAMIC			(1)

/* ------------------------------------- Go Home config ------------------------------------- */

/* Set Home cells list size */
#define HOME_POINTS_SIZE		(3)

/* ------------------------------------- Gyro config------------------------------------- */

/*
 * Gyro dynamic setting. If on, it will enable gyro dynamic adjustment while robot turning.
 */
#define GYRO_DYNAMIC_ADJUSTMENT	(1)

// Define the position of gyro, only one side could be chozen.
#define GYRO_FRONT_X_POS	(1)
#define GYRO_FRONT_X_NEG	(0)
#define GYRO_FRONT_Y_POS	(0)
#define GYRO_FRONT_Y_NEG	(0)

/*
 * Defines for enabling/disabling tilted detect or not.
 * When detecting the robot is tilted, it will check the all the 3 cliff
 * sensors value, which if all are less then 1500, and at the same time,
 * if the gyro x-axis & y-axis angle are greater than 5 degree, we will
 * confirmed that the robot is tilted.
 */
#define ENABLE_TILTED_DETECT		(1)

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

/* ------------------------------------- Movement config ------------------------------------- */

/*
 * Range of LINEAR_MIN_SPEED should be better within 12 to 15.
 * When it is too small, it will move like shaking when robot startup.
 * When it is too large, it will fall down when reach the cliff.
 */
#define SPEED_ALF						(7.83)
#define BACK_MAX_SPEED					((int32_t) 11) // 15)
#define BACK_MIN_SPEED					((int32_t) 8)
#define LINEAR_MIN_SPEED				((int32_t) 10) // 15)
#define LINEAR_MAX_SPEED				((int32_t) 38) // 15)
#define FALL_WALL_MIN_SPEED				((int32_t) 5)
#define FALL_WALL_MAX_SPEED				((int32_t) 20)
#define ROTATE_TOP_SPEED				((uint8_t) 20) // 22)
#define ROTATE_LOW_SPEED				((uint8_t) 8)
#define RUN_TOP_SPEED					((int32_t) 38) // 45)

#define LINEAR_NEAR_DISTANCE			(CELL_SIZE*1.5)

/* ------------------------------------- Slam config ------------------------------------- */
#define USE_ROBOT_TF						(1)

/* ------------------------------------- Lidar config ------------------------------------- */
#define LIDAR_FOLLOW_WALL					(1)

#define LIDAR_MARKER						(1)

#define LIDAR_THETA							(205*PI/180)//2050
#define LIDAR_OFFSET_X						(0.031)//0.031
#define LIDAR_OFFSET_Y						(0)//0
#define ROBOT_RADIUS						(0.167)

/* ------------------------------------- robot physical config ------------------------------------- */
#define WHEEL_TO_CENTER_DISTANCE		(0.109f)

#define WHEEL_ENCODER_TO_MILLIMETER			(0.1955f)
#endif //__CONFIG_H__

