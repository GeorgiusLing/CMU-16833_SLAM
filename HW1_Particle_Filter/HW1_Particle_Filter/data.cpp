#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "data.hpp"

cv::Mat Mapper::readMap(std::string filename) {
	std::string line;
	std::ifstream file(filename);
	if (!file)
	{
		std::cout << "File not found.";
		return this->grid;
	}
	std::cout << "Loading map..." << std::endl;

	bool is_header = true;

	int num_rows, num_cols;
	int width, height;
	int index = 0;
	while (std::getline(file, line))
	{
		if (line.length() < 1) {
			continue;
		}
		std::stringstream linestream(line);
		if (is_header) {

			std::string prefix;
			linestream >> prefix;
			if (prefix.compare("robot_specifications->global_mapsize_x") == 0) {
				linestream >> width;
			}
			else if (prefix.compare("robot_specifications->global_mapsize_y") == 0) {
				linestream >> height;
			}
			else if (prefix.compare("robot_specifications->resolution") == 0) {
				linestream >> this->resolution;
			}
			else if (prefix.compare("global_map[0]:") == 0) {
				linestream >> num_rows >> num_cols;
				is_header = false;
				this->grid = cv::Mat(num_rows, num_cols, CV_32F);
			}
		}
		else {
			for (int i = 0; i < num_cols; i++) {
				float entry;
				linestream >> entry;
				grid.at<float>(index, i) = entry;
			}
			++index;
		}
	}
	cv::Mat copy;
	cv::resize(grid, copy, cv::Size(height, width));
	std::cout << "Finish loading " + filename << ", height: " << copy.rows << ", width: " << copy.cols << std::endl;
	return copy;
}

void Mapper::drawMap(std::string winname) {
	cv::imshow(winname, grid);
}

std::queue<RobotDatum> ReadLog(std::string filename) {
	std::ifstream ifs(filename);
	std::string line;
	std::queue<RobotDatum> data;
	if (!ifs) {
		std::cout << "Cannot find the file!" << std::endl;
		return data;
	}
	while (std::getline(ifs, line)) {
		std::stringstream linestream(line);
		char prefix;
		linestream >> prefix;
		if (prefix == 'O') {
			RobotDatum datum;
			datum.type = ODOM;
			linestream >> datum.center0[0] >> datum.center0[1] >> datum.heading0;
			linestream >> datum.timestamp;
			data.push(datum);
		}
		else if (prefix == 'L') {
			RobotDatum datum;
			datum.type = SENSOR;
			linestream >> datum.center0[0] >> datum.center0[1] >> datum.heading0 >> datum.center1[0] >> datum.center1[1] >> datum.heading1;
			datum.measurements = new float[180];
			for (int i = 0; i < 180; i++) {
				linestream >> datum.measurements[i];
			}
			linestream >> datum.timestamp;
			data.push(datum);
		}
		else {
			continue;
		}
	}
	std::cout << "Successfully read logged data." << std::endl;
	return data;
}