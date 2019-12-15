#pragma once

/*
 * data.hpp
 *
 *  Created on: Mar 31, 2019
 *      Author: linzh
 */


#ifndef DATA_HPP_
#define DATA_HPP_

#include <stdlib.h>
#include <vector>
#include <queue>
#include <string>

#include <opencv2/core/core.hpp>

enum DataType {
	ODOM, SENSOR,
};

struct RobotDatum {
	DataType type;
	cv::Vec2f center0;
	float heading0;    // orientation of robot
	float timestamp;
	cv::Vec2f center1;
	float heading1;    // orientation of sensor
	float *measurements;
};

class Mapper {
public:
	Mapper(void) = default;
	~Mapper(void) = default;
	cv::Mat readMap(std::string filename);
	void drawMap(std::string winname);
private:
	cv::Mat grid;
	int resolution;
};


std::queue<RobotDatum> readLog(std::string filename);

#endif /* DATA_HPP_ */
