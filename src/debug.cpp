#include <stdint.h>
#include <stdio.h>

#include <ros/ros.h>

#include "debug.h"
#include "path_planning.h"
#include "core_move.h"

char outString[256];

extern Cell_t g_cell_history[];

#if defined(DEBUG_MAP) || defined(DEBUG_SM_MAP)
#if COLOR_DEBUG_MAP
void color_print(char *outString,int16_t y_min,int16_t y_max)
{
	int16_t j = 0;
	char cs;
	bool ready_print_map = 0;
	for(j =y_min; j<=y_max; j++){
		cs = *(outString+j);
		if(cs =='\t' && !ready_print_map){
			ready_print_map = 1;
			printf("\t");
			continue;
		}
		else if(!ready_print_map){
			printf("%c",cs);
			continue;
		}
		if(ready_print_map){
			if(cs == '0'){//unclean
				printf("\033[0;40;37m""%c""\033[0m",cs);//bright black
			}
			else if(cs == '1'){//clean
				printf("\033[1;42;37m""%c""\033[0m",cs);//green
			}
			else if(cs == '2'){//bumper
				printf("\033[1;44;37m""%c""\033[0m",cs);//blue
			}
			else if(cs == '3'){//obs
				printf("\033[1;41;37m""%c""\033[0m",cs);//red
			}
			else if(cs == '4'){//cliff
				printf("\033[1;45;37m""%c""\033[0m",cs);//pink
			}
			else if(cs == '5'){//rcon
				printf("\033[1;46;37m""%c""\033[0m",cs);//deep green
			}
			else if(cs == '6'){//tilt
				printf("\033[1;47;30m""%c""\033[0m",cs);//white black
			}
			else if(cs == '7'){//boudary
				printf("\033[1;40;37m""%c""\033[0m",cs);//black
			}
			else if(cs == 'e'){//end point
				printf("\033[5;43;37m""%c""\033[0m",cs);//blink white yellow
			}
			else if(cs == 'x'){//cur point
				printf("\033[5;45;37m""%c""\033[0m",cs);//blink red
			}
			else if(cs == '>'){//target point
				printf("\033[5;47;30m""%c""\033[0m",cs);//white
			}
			else{
				printf("%c ",cs);
			}
		}
	}
	printf("\n");
}
#endif
/*
 * Function to print the robot cleaning map.
 *
 * @param id    MAP id, 0 for cleaning map, 1 for shortest path map.
 * @param endx  X coordinate of target
 * @param endy  Y coordinate of target
 *
 * @return
 */
void debug_map(uint8_t id, int16_t endx, int16_t endy)
{
#if ENABLE_DEBUG
	int16_t		i, j, x_min, x_max, y_min, y_max, index;
	CellState	cs;
	Cell_t temp_cell;

	if (g_trapped_mode == 1)
		temp_cell = map_get_curr_cell();
	else
		temp_cell = g_cell_history[0];

	path_get_range(&x_min, &x_max, &y_min, &y_max);

	ROS_INFO("Map: %s", id == MAP ? "MAP" : "SPMAP");
	index = 0;
	outString[index++] = '\t';
	for (j = y_min; j <= y_max; j++) {
		if (abs(j) % 10 == 0) {
			outString[index++] = (j < 0 ? '-' : ' ');
#if COLOR_DEBUG_MAP
			//outString[index++] = ' ';//
#endif
			outString[index++] = (abs(j) >= 100 ? abs(j) / 100 + 48 : ' ');
#if COLOR_DEBUG_MAP
			//outString[index++] = ' ';//
#endif
			outString[index++] = 48 + (abs(j) >= 10 ? ((abs(j) % 100) / 10) : 0);
#if COLOR_DEBUG_MAP
			//outString[index++] = ' ';//
#endif
			j += 3;
		} else {
			outString[index++] = ' ';
#if COLOR_DEBUG_MAP
			//outString[index++] = ' ';//
#endif
		}
	}
	outString[index++] = 0;

#if COLOR_DEBUG_MAP
	printf("%s\n", outString);
#else
	ROS_INFO("%s",outString);
#endif
	index = 0;
	outString[index++] = '\t';
	for (j = y_min; j <= y_max; j++) {
		outString[index++] = abs(j) % 10 + 48;
#if COLOR_DEBUG_MAP
		//outString[index++] = ' ';//
#endif
	}
	outString[index++] = 0;
#if COLOR_DEBUG_MAP
	//outString[index++] = ' ';//
	printf("%s\n\n", outString);
	memset(outString,0,256);
#else
	ROS_INFO("%s\n",outString);
#endif
	for (i = x_min; i <= x_max; i++) {
		index = 0;

		outString[index++] = (i < 0 ? '-' : ' ');
		outString[index++] = 48 + (abs(i) >= 100 ? abs(i) / 100 : 0);
		outString[index++] = 48 + (abs(i) >= 10 ? ((abs(i) % 100) / 10) : 0);
		outString[index++] = abs(i) % 10 + 48;
		outString[index++] = '\t';

		for (j = y_min; j <= y_max; j++) {
			cs = map_get_cell(id, i, j);
			if (i == temp_cell.X && j == temp_cell.Y) {
				outString[index++] = 'x';
			} else if (i == endx && j == endy) {
				outString[index++] = 'e';
			} else {
				outString[index++] = cs + 48;
			}
		}
#if COLOR_DEBUG_MAP 
		color_print(outString, 0,index);
#else
		ROS_INFO("%s", outString);
#endif
	}
	ROS_INFO("\n");
#endif
}
#endif
