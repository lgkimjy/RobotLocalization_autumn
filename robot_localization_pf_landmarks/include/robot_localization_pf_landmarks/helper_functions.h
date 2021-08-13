/*
 * helper_functions.h
 *      Author: junyoung kim / lgkimjy
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "map.h"

inline double dist(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline bool read_map_data(std::string filename, Map &map)
{
	// Get file of map:
	std::ifstream in_file_map(filename.c_str(), std::ifstream::in);
	if (!in_file_map) return false;

	std::string line_map;

	while (getline(in_file_map, line_map))
	{

		std::istringstream iss_map(line_map);
		float landmark_x_f, landmark_y_f;
		int id_i;

		// Read data from current line to values::
		iss_map >> landmark_x_f;
		iss_map >> landmark_y_f;
		iss_map >> id_i;

		Map::single_landmark_s single_landmark_temp;

		single_landmark_temp.id_i = id_i;
		single_landmark_temp.x_f = landmark_x_f;
		single_landmark_temp.y_f = landmark_y_f;

		map.landmark_list.push_back(single_landmark_temp);
	}
	return true;
}

#endif /* HELPER_FUNCTIONS_H_ */