#pragma once

#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <stdlib.h>
#include <vector>
#include <set>
#include <queue>
#include <string>
#include <random>

#include <opencv2/core/mat.hpp>

#include "data.hpp"

class MotionModel {
public:
    double param1;
    double param2;
    double param3;
    double param4;

    MotionModel(void);
    void updatePose(cv::Vec4d &particle);
    void updateOdom(RobotDatum &datum);

private:
    cv::Vec3d pose;
    double displacement;
    double rotation1;
    double rotation2;
    double timestamp;
    cv::RNG rng;
};

class SensorModel {
public:
    double param_hit;
    double param_max;
    double param_rand;
    double param_short;
    double lambda_short;
    double sigma_hit;
    int range;
    void updateWeight(const cv::Mat &inoccupancy, const double &threshold, cv::Vec4d &particle, std::vector<int> measurements, int step = 1);

private:
    double calcLikelohood(int reading, double expected);
};

class ParticleFilter {
public:
    SensorModel sensor_model;
    MotionModel odometry_model;

    void setMap(cv::Mat &grid, float threshold = 0.5);
    void sample(unsigned int num_particles);
    void update(RobotDatum &datum);
    void resample(void);
    void visualize(std::string winname, double fx, double fy);

private:
    cv::Mat inoccupancy;
    float threshold;
    std::vector<double> comb;
    std::vector<cv::Vec4d> particles;
    const cv::Size axes = cv::Size(50, 20);
    const cv::Scalar color = cv::Scalar(0.047, 0.047, 0.949);
};

#endif /* FILTER_HPP_ */
