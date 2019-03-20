#include <iostream>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <path_gen_srv/path_gen_srv.h>
#include <nav_msgs/GetMap.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "dynamic_reconfigure/server.h"
#include <path_gen_srv/path_srvConfig.h>

using namespace std;
using namespace cv;
// define size of the map.(In ros,4000x4000 is default setting)
#define Height 1024
#define Width 1024
// define size of openset and closedset
#define OpenSet_array_size 2000
#define closedset_array_size 40000
// define mark of obstacle and unknown place on the map
#define obstacle_mark 100
#define unknown_mark -1

// define cost of moving to next grid(only normal moving)
#define cost 10
// define openset and closedset index
#define row 0
#define column 1
#define fn 2
#define parent_row 3
#define parent_col 4
#define valid_bit 5
#define gn 6
// define valid macro of valid_bit
#define valid 1
#define invalid 0
// define maximum path length
#define length 1000
// define macro of gn calculation
#define hypotenous 1
#define normal 0
// define two bias for coordinate transformation

float x_bias = 0;
float y_bias = 0;
int8_t map_arr[Height][Width];
Mat cost_map;

uint16_t max_openset_index = 0;
nav_msgs::GetMap get_map;

float cost_scalar = 0.7;
// define heuristic function gain,it can be [0,inf]
float heuristic_scaler = 1;
/*
	error code  0: no sloution to the map
	error code -1:
	error code -2: start point is coincident to target
*/
uint16_t A_star_path_finding(int8_t map[Height][Width], uint16_t start_point[2], uint16_t end_point[2], uint16_t Path[length][fn]);
inline int Get_heuristic_function(uint16_t current_point[2], uint16_t Goal[2]);
int check_end_point_in_OpenSet(uint16_t end_point[2], uint16_t OpenSet[OpenSet_array_size][7]);
uint32_t Get_minimun_fn_coord(uint16_t OpenSet[OpenSet_array_size][7]);
uint16_t Get_gn(uint16_t current_point_gn, char dir);
void Expend_current_point(int8_t map[Height][Height], uint16_t current_point[3], uint16_t OpenSet[OpenSet_array_size][7], uint16_t ClosedSet[closedset_array_size][7], uint16_t end_point[2], uint16_t start_point[2], uint16_t closed_list_counter);
bool Is_OpenSet_empty(uint16_t OpenSet[OpenSet_array_size][7]);
void Init(uint16_t OpenSet[OpenSet_array_size][7]);
bool Is_in_closed_set(uint16_t point[2], uint16_t ClosedSet[closedset_array_size][7], uint16_t closed_list_counter);
short Is_in_Open_set(uint16_t point[2], uint16_t Openset[OpenSet_array_size][7]);
uint16_t Find_parent_index(uint16_t parent[2], uint16_t ClosedSet[closedset_array_size][7], uint16_t CloseSet_index);
uint16_t Get_path(uint16_t ClosedSet[closedset_array_size][7], uint16_t CloseSet_index, uint16_t Path[length][fn], uint16_t start_point[2]);
uint16_t Check_start_end_point(int8_t map[Height][Width], uint16_t start_point[2], uint16_t end_point[2]);
void Transform_coordinate(float start[2], float end[2], uint16_t start_point[2], uint16_t end_point[2], const float *map_resolution);
bool map_rec_callback(path_gen_srv::path_gen_srv::Request &req, path_gen_srv::path_gen_srv::Response &res);
bool Is_in_open_or_closed_set(uint16_t point[2], uint16_t OpenSet[OpenSet_array_size][7], uint16_t ClosedSet[closedset_array_size][7], uint16_t closed_list_counter);
inline float Get_cost_map_value(uint16_t point[2]);
void inverse_color(void);
void reconfigure_callback(path_gen_srv::path_srvConfig &config, uint32_t level);
void find_avaiable_point(uint16_t point[2]);

void reconfigure_callback(path_gen_srv::path_srvConfig &config, uint32_t level)
{
	cost_scalar = config.cost_scalar;
	heuristic_scaler = config.heuristic_scaler;
	ROS_INFO("cost scalar:%f  heuristic scalar:%f", cost_scalar, heuristic_scaler);
}
// path finding function,can be alternate to other algorithm like Dijstra
uint16_t A_star_path_finding(int8_t map[Height][Width], uint16_t start_point[2], uint16_t end_point[2], uint16_t Path[length][fn])
{
	// Set {row_coordinate,column_coodrinate,f(n),x_parent,y_parent,valid_bit,gn}
	uint16_t ClosedSet[closedset_array_size][7];
	uint16_t OpenSet[OpenSet_array_size][7];
	uint16_t current_point[3]; // row col gn
	uint16_t min_index = 0;
	uint16_t CloseSet_index = 0;
	uint16_t Path_length = 0;

	Init(OpenSet);

	OpenSet[0][row] = start_point[row];
	OpenSet[0][column] = start_point[column];
	OpenSet[0][fn] = Get_heuristic_function(start_point, end_point);
	OpenSet[0][parent_row] = start_point[row];
	OpenSet[0][parent_col] = start_point[column];
	OpenSet[0][valid_bit] = valid;
	OpenSet[0][gn] = 0;
	while (check_end_point_in_OpenSet(end_point, OpenSet) == -1)
	{
		if (!Is_OpenSet_empty(OpenSet))
		{
			min_index = Get_minimun_fn_coord(OpenSet);
			current_point[row] = OpenSet[min_index][row];
			current_point[column] = OpenSet[min_index][column];
			current_point[2] = OpenSet[min_index][gn];

			ClosedSet[CloseSet_index][row] = OpenSet[min_index][row];
			ClosedSet[CloseSet_index][column] = OpenSet[min_index][column];
			ClosedSet[CloseSet_index][fn] = OpenSet[min_index][fn];
			ClosedSet[CloseSet_index][parent_row] = OpenSet[min_index][parent_row];
			ClosedSet[CloseSet_index][parent_col] = OpenSet[min_index][parent_col];
			ClosedSet[CloseSet_index][valid_bit] = valid;

			OpenSet[min_index][valid_bit] = invalid;
			CloseSet_index++;
			Expend_current_point(map, current_point, OpenSet, ClosedSet, end_point, start_point, CloseSet_index);
		}
		else
		{
			return 0;
		}
	}
	int index = check_end_point_in_OpenSet(end_point, OpenSet);
	ClosedSet[CloseSet_index][row] = OpenSet[index][row];
	ClosedSet[CloseSet_index][column] = OpenSet[index][column];
	ClosedSet[CloseSet_index][fn] = OpenSet[index][fn];
	ClosedSet[CloseSet_index][parent_row] = OpenSet[index][parent_row];
	ClosedSet[CloseSet_index][parent_col] = OpenSet[index][parent_col];
	ClosedSet[CloseSet_index][valid_bit] = valid;
	OpenSet[index][valid_bit] = invalid;
	CloseSet_index++;
	Path[0][row] = end_point[row];
	Path[0][column] = end_point[column];
	Path_length = Get_path(ClosedSet, CloseSet_index, Path, start_point);
	return Path_length;
}

// Get path
uint16_t Get_path(uint16_t ClosedSet[closedset_array_size][7], uint16_t CloseSet_index, uint16_t Path[length][fn], uint16_t start_point[2])
{
	uint16_t Path_index = 1;
	uint16_t parent[2];
	uint16_t parent_index = 0;
	parent[row] = ClosedSet[CloseSet_index - 1][parent_row];
	parent[column] = ClosedSet[CloseSet_index - 1][parent_col];
	while (!(Path[Path_index - 1][row] == start_point[row] && Path[Path_index - 1][column] == start_point[column]))
	{
		parent_index = Find_parent_index(parent, ClosedSet, CloseSet_index);
		Path[Path_index][row] = ClosedSet[parent_index][row];
		Path[Path_index][column] = ClosedSet[parent_index][column];
		parent[row] = ClosedSet[parent_index][parent_row];
		parent[column] = ClosedSet[parent_index][parent_col];
		Path_index++;
	}
	return Path_index;
}

// Find parent coordinate index from closed set
uint16_t Find_parent_index(uint16_t parent[2], uint16_t ClosedSet[closedset_array_size][7], uint16_t CloseSet_index)
{
	for (int i = CloseSet_index - 1; i >= 0; i--)
	{
		if (parent[row] == ClosedSet[i][row] && parent[column] == ClosedSet[i][column])
		{
			return i;
		}
	}
	return 0;
}

//initialzation of close set and open set,just set valid_bit to 0
void Init(uint16_t OpenSet[OpenSet_array_size][7])
{
	for (uint16_t i = 0; i < OpenSet_array_size; i++)
	{
		OpenSet[i][valid_bit] = 0;
	}
	max_openset_index = 0;
}

// check if the point is in the open set or closed set.
bool Is_in_closed_set(uint16_t point[2], uint16_t ClosedSet[closedset_array_size][7], uint16_t closed_list_counter)
{
	for (int i = 0; i < closed_list_counter; i++)
	{
		if (ClosedSet[i][row] == point[0] && ClosedSet[i][column] == point[1])
		{
			return true;
		}
	}
	return false;
}

// check if OpenSet is empty or not
bool Is_OpenSet_empty(uint16_t OpenSet[OpenSet_array_size][7])
{
	for (int i = 0; i < OpenSet_array_size; i++)
	{
		if (OpenSet[i][valid_bit] == 1)
			return false;
	}
	return true;
}

// check if point is in openset,if it is, return index ,else return -1
short Is_in_Open_set(uint16_t point[2], uint16_t Openset[OpenSet_array_size][7])
{
	for (int i = 0; i < max_openset_index; i++)
	{
		if (Openset[i][row] == point[0] && Openset[i][column] == point[1] && Openset[i][valid_bit] == valid)
		{
			return i;
		}
	}
	return -1;
}

bool Is_in_open_or_closed_set(uint16_t point[2], uint16_t OpenSet[OpenSet_array_size][7], uint16_t ClosedSet[closedset_array_size][7], uint16_t closed_list_counter)
{
	for (int i = 0; i < max_openset_index; i++)
	{
		if (OpenSet[i][valid_bit] == 1 && (OpenSet[i][row] == point[0] && OpenSet[i][column] == point[1]))
		{
			return true;
		}
	}
	for (int i = 0; i < closed_list_counter; i++)
	{
		if (ClosedSet[i][row] == point[0] && ClosedSet[i][column] == point[1])
		{
			return true;
		}
	}
	return false;
}

inline float Get_cost_map_value(uint16_t point[2])
{
	return cost_map.data[point[0] * Width + point[1]] * cost_scalar;
}

void Expend_current_point(int8_t map[Height][Height], uint16_t current_point[3], uint16_t OpenSet[OpenSet_array_size][7], uint16_t ClosedSet[closedset_array_size][7], uint16_t end_point[2], uint16_t start_point[2], uint16_t closed_list_counter)
{

	uint16_t point[2];
	int index = 0;
	// upper
	point[0] = current_point[0] - 1;
	point[1] = current_point[1];

	if (current_point[0] > 0 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != unknown_mark && map[point[0]][point[1]] != obstacle_mark)
	{
		// create a point variable
		index = Is_in_Open_set(point, OpenSet);

		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], normal) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], normal);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], normal);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}
	// upper left
	point[0] = current_point[0] - 1;
	point[1] = current_point[1] - 1;

	if (current_point[1] > 0 && current_point[0] > 0 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark && map[point[0]][point[1]] != unknown_mark)
	{
		// create a point variable
		index = Is_in_Open_set(point, OpenSet);
		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], hypotenous) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}

	// upper right
	point[0] = current_point[0] - 1;
	point[1] = current_point[1] + 1;

	if (current_point[1] < Width - 1 && current_point[0] > 0 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark && map[point[0]][point[1]] != unknown_mark)
	{
		// create a point variable
		index = Is_in_Open_set(point, OpenSet);
		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], hypotenous) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}

	// left
	point[0] = current_point[0];
	point[1] = current_point[1] - 1;

	if (current_point[1] > 0 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark && map[point[0]][point[1]] != unknown_mark)
	{
		// create a point variable
		index = Is_in_Open_set(point, OpenSet);

		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], normal) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], normal);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], normal);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}
	// right
	point[0] = current_point[0];
	point[1] = current_point[1] + 1;

	if (current_point[1] < Width - 1 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark && map[point[0]][point[1]] != unknown_mark)
	{

		// create a point variable
		index = Is_in_Open_set(point, OpenSet);

		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], normal) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], normal);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], normal);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}
	// down
	point[0] = current_point[0] + 1;
	point[1] = current_point[1];
	if (current_point[0] < Height - 1 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark && map[point[0]][point[1]] != unknown_mark)
	{
		// create a point variable
		index = Is_in_Open_set(point, OpenSet);

		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], normal) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], normal);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], normal);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}

	// lower left
	point[0] = current_point[0] + 1;
	point[1] = current_point[1] - 1;
	if (current_point[1] > 0 && current_point[0] < Height - 1 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark && map[point[0]][point[1]] != unknown_mark)
	{
		// create a point variable
		index = Is_in_Open_set(point, OpenSet);
		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], hypotenous) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}

	// lower right
	point[0] = current_point[0] + 1;
	point[1] = current_point[1] + 1;

	if (current_point[1] < Width - 1 && current_point[0] < Height - 1 && !Is_in_closed_set(point, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark && map[point[0]][point[1]] != unknown_mark)
	{
		// create a point variable
		index = Is_in_Open_set(point, OpenSet);
		if (index != -1) // point is existed
		{
			if (Get_gn(current_point[2], hypotenous) < OpenSet[index][gn])
			{
				OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
				OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
				OpenSet[index][parent_row] = current_point[row];
				OpenSet[index][parent_col] = current_point[column];
				OpenSet[index][valid_bit] = valid;
			}
		}
		else // point is not exist in openset
		{
			for (int i = 0; i < OpenSet_array_size; i++)
			{
				if (OpenSet[i][valid_bit] == 0)
				{
					index = i;
					break;
				}
				else if (i == OpenSet_array_size)
				{
					cout << "open list fulled." << endl;
				}
			}
			OpenSet[index][row] = point[row];
			OpenSet[index][column] = point[column];
			OpenSet[index][gn] = Get_gn(current_point[2], hypotenous);
			OpenSet[index][fn] = Get_heuristic_function(point, end_point) + OpenSet[index][gn] + Get_cost_map_value(point);
			OpenSet[index][parent_row] = current_point[row];
			OpenSet[index][parent_col] = current_point[column];
			OpenSet[index][valid_bit] = valid;
			if (index > max_openset_index)
			{
				max_openset_index = index;
			}
		}
	}
}
// get Manhaton distance
inline int Get_heuristic_function(uint16_t current_point[2], uint16_t Goal[2])
{
	//return sqrt((current_point[0] - Goal[0]) * (current_point[0] - Goal[0]) + (current_point[1] - Goal[1]) * (current_point[1] - Goal[1])) * cost * heuristic_scaler;
	return (abs(Goal[1] - current_point[1]) + abs(Goal[0] - current_point[0])) * cost * heuristic_scaler;
}

uint16_t Get_gn(uint16_t current_point_gn, char dir)
{
	static const int cost_hypo = sqrt(cost * cost + cost * cost);
	uint16_t gn_ = 0;
	switch (dir)
	{
	case normal:
		gn_ = current_point_gn + cost;
		break;
	case hypotenous:
		gn_ = current_point_gn + cost_hypo;
		break;
	}
	return gn_;
}

int check_end_point_in_OpenSet(uint16_t end_point[2], uint16_t OpenSet[OpenSet_array_size][7])
{
	for (uint16_t i = 0; i < max_openset_index; i++)
	{
		if (OpenSet[i][row] == end_point[0] && OpenSet[i][column] == end_point[1])
			return i;
	}
	return -1;
}

uint32_t Get_minimun_fn_coord(uint16_t OpenSet[OpenSet_array_size][7])
{
	uint32_t result_index = 0;
	uint16_t minimun = 0;
	uint16_t i = 0;
	for (i = 0; i < OpenSet_array_size; i++)
	{
		if (OpenSet[i][valid_bit] == 1)
		{
			minimun = OpenSet[i][fn];
			result_index = i;
			break;
		}
	}
	for (; i < OpenSet_array_size; i++)
	{
		if (OpenSet[i][fn] < minimun && OpenSet[i][valid_bit] == 1)
		{
			result_index = i;
			minimun = OpenSet[i][fn];
		}
	}
	return result_index;
}

// Here to check if the start point and end point are coincident with any obstacle
uint16_t Check_start_end_point(int8_t map[Height][Width], uint16_t start_point[2], uint16_t end_point[2])
{
	uint16_t code = 0;
	if (map[start_point[0]][start_point[1]] == obstacle_mark)
		code |= 0x01;
	if (map[end_point[0]][end_point[1]] == obstacle_mark)
		code |= 0x10;
	if (start_point[0] == end_point[0] && start_point[1] == end_point[1])
		code |= 0x100;
	return code;
}

void Transform_coordinate(float start[2], float end[2], uint16_t start_point[2], uint16_t end_point[2], const float *map_resolution)
{
	start_point[0] = (uint16_t)((start[0] - y_bias) / *map_resolution);
	start_point[1] = (uint16_t)((start[1] - x_bias) / *map_resolution);
	end_point[0] = (uint16_t)((end[0] - y_bias) / *map_resolution);
	end_point[1] = (uint16_t)((end[1] - x_bias) / *map_resolution);
}

void inverse_color(void)
{
	for (int i = 0; i < Height; i++)
	{
		for (int j = 0; j < Width; j++)
		{
			cost_map.data[i * Width + j] = 255 - cost_map.data[i * Width + j];
		}
	}
}

void find_avaiable_point(uint16_t point[2])
{
	uint8_t expend_index = 0;
	do
	{
		expend_index++;
		for (int i = -expend_index; i <= expend_index; i++)
		{
			if (map_arr[point[0] - expend_index][point[1] + i] != obstacle_mark && map_arr[point[0] - expend_index][point[1] + i] != unknown_mark) // ->
			{
				point[0] = point[0] - expend_index;
				point[1] = point[1] + i;
				return;
			}
			if (map_arr[point[0] + i][point[1] - expend_index] != obstacle_mark && map_arr[point[0] + i][point[1] - expend_index] != unknown_mark) // go down
			{
				point[0] = point[0] + i;
				point[1] = point[1] - expend_index;
				return;
			}
		}
		for (int i = -expend_index + 1; i <= expend_index; i++)
		{
			if (map_arr[point[0] + expend_index][point[1] + i] != obstacle_mark && map_arr[point[0] + expend_index][point[1] + i] != unknown_mark) // ->
			{
				point[0] = point[0] - expend_index;
				point[1] = point[1] + i;
				return;
			}
			if (map_arr[point[0] + i][point[1] + expend_index] != obstacle_mark && map_arr[point[0] + i][point[1] + expend_index] != unknown_mark) // go down
			{
				point[0] = point[0] + i;
				point[1] = point[1] - expend_index;
				return;
			}
		}
	} while (expend_index < 200);
}

bool map_rec_callback(path_gen_srv::path_gen_srv::Request &req, path_gen_srv::path_gen_srv::Response &res)
{

	uint16_t Path[length][2];							 // to store a path, can be changed to some variables array with no length declaration
	clock_t start_time, end_time;					 // to calculate how long the algorithm takes
	float start[2], end[2];								 // received start and goal(assumed)
	uint16_t start_point[2], end_point[2]; // transformed start point and end point
	uint16_t Path_length;
	nav_msgs::Path path;

	start[0] = req.start_point.position.y; // row,which is y in grid map coordinate
	start[1] = req.start_point.position.x;

	end[0] = req.goal.position.y;
	end[1] = req.goal.position.x;

	ROS_INFO("%f %f %f %f", start[0], start[1], end[0], end[1]);

	Transform_coordinate(start, end, start_point, end_point, &get_map.response.map.info.resolution);
	ROS_INFO("%d %d %d %d", start_point[0], start_point[1], end_point[0], end_point[1]);
	uint16_t error_code = Check_start_end_point(map_arr, start_point, end_point);
	bool search_enable = false;
	if (error_code == 0)
		search_enable = true;
	if (error_code == 0x001)
	{
		ROS_ERROR("Error:start point is coincident with obstacle.");
		ROS_INFO("searching the closest avaiable point");
		find_avaiable_point(start_point);
		search_enable = true;
	}
	else if (error_code == 0x010)
	{
		ROS_ERROR("Error:end point is coincident with obstacle.");
		ROS_INFO("searching the closest avaiable point");
		find_avaiable_point(end_point);
		search_enable = true;
	}
	else if (error_code == 0x011)
	{
		ROS_ERROR("Error:two point are coincident with obstacles.");
		ROS_INFO("searching the closest avaiable point");
		find_avaiable_point(start_point);
		find_avaiable_point(end_point);
		search_enable = true;
	}
	if (search_enable == true)
	{
		ROS_INFO("start_grid:(%d,%d) end_grid:(%d %d)", start_point[0], start_point[1], end_point[0], end_point[1]);
		ROS_INFO("%d", map_arr[end_point[0]][end_point[1]]);
		ROS_INFO("Start searching...");
		start_time = clock();																											// start the clock ticking
		Path_length = A_star_path_finding(map_arr, start_point, end_point, Path); // core function
		end_time = clock();																												// stop ticking clock
		ROS_INFO("Searching complete.");
		switch (Path_length)
		{
		case 0:
			ROS_INFO("No solution to the map");
			break;
		default:
			ROS_INFO("Total runtime:%f,There is %d steps in total.", (double)(end_time - start_time) / CLOCKS_PER_SEC, Path_length);

			geometry_msgs::PoseStamped poses[Path_length];
			path.header.frame_id = "map";
			path.header.stamp = ros::Time::now();

			for (int i = 0; i < Path_length; i++)
			{
				poses[i].header.frame_id = "map";
				poses[i].header.stamp = ros::Time::now();
				poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0);
				poses[i].pose.position.x = get_map.response.map.info.resolution * Path[i][1] + x_bias;
				poses[i].pose.position.y = get_map.response.map.info.resolution * Path[i][0] + y_bias;
				poses[i].pose.position.z = 0;
				path.poses.push_back(poses[i]);
			}
			res.Path = path;
		}
		ROS_INFO("Sending back path...");
		cout << endl;
	}
	else
	{
		ROS_ERROR("not good.");
		path.poses.clear();
		res.Path = path;
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_finding_node");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("/costmap/static_map");
	dynamic_reconfigure::Server<path_gen_srv::path_srvConfig> dynamic_server;
	dynamic_reconfigure::Server<path_gen_srv::path_srvConfig>::CallbackType f;
	f = boost::bind(&reconfigure_callback, _1, _2);
	dynamic_server.setCallback(f);

	while (!client.call(get_map))
	{
		ROS_INFO("service call failed,call again.");
	}
	uint32_t counter = 0;
	for (int i = 0; i < Height; i++)
	{
		for (int j = 0; j < Width; j++)
		{
			map_arr[i][j] = get_map.response.map.data[counter];
			counter++;
		}
	}
	x_bias = get_map.response.map.info.origin.position.x;
	y_bias = get_map.response.map.info.origin.position.y;
	ROS_INFO("map copy done.");
	Mat eroded_map = imread("/home/alcatraz/catkin_ws/src/multiple_rb_ctrl/maps/eroded_map.pgm", 0);
	GaussianBlur(eroded_map, cost_map, Size(21, 15), 9, 9);
	inverse_color();
	ROS_INFO("map aquired,resolution:%f", get_map.response.map.info.resolution);
	ros::ServiceServer server = nh.advertiseService("path_server", map_rec_callback);
	ROS_INFO("Path finder ready!");
	ros::spin();
	ROS_INFO("Program exiting.");
	return 0;
}
