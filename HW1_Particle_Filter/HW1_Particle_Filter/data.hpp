#pragma once

/*
 * data.hpp
 *
 *  In this header file some classes and functions
 *  used to process different data types are declared.
 */


#ifndef DATA_HPP_
#define DATA_HPP_

#include <cstdlib>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

struct RobotDatum {
	char type;
	cv::Vec3d pose0;    // pose of the robot
	
	cv::Vec3d pose1;    // pose of the sensor
	std::vector<int> measurements;
    
    double timestamp;
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

std::vector<RobotDatum> readLog(std::string filename);

#endif /* DATA_HPP_ */
