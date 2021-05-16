/**
    16833 Assignment 1: Particle Filter (re-written)
    @file MapReader.cpp
    @author Lin, Zhaozhi
    @date Dec 2019
*/


#include "ParticleFilter.hpp"

MapReader::MapReader(const std::string& filename) {
    std::string line;
    std::ifstream file(filename);
    if (!file)
    {
        throw std::runtime_error("File " + filename + " not found");
    }
    std::cout << "Loading map..." << std::endl;

    bool isHeader = true;
    int numRows, numCols;
    int width, height;
    int index = 0;
    while (std::getline(file, line))
    {
        if (line.length() < 1) {
            continue;
        }
        std::stringstream linestream(line);
        if (isHeader) {
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
                linestream >> numRows >> numCols;
                isHeader = false;
                index = numRows - 1;
                this->occupancyMap = cv::Mat(numRows, numCols, CV_32F);
            }
        }
        else {
            for (int i = 0; i < numCols; i++) {
                float entry;
                linestream >> entry;
                occupancyMap.at<float>(index, i) = entry;
            }
            --index;
        }
    }
}

cv::Mat MapReader::getMap(void) const {
    
    return this->occupancyMap;
}

size_t MapReader::getResolution(void) const {
    return this->resolution;
}