/**
    16833 Assignment 1: Particle Filter (re-written)
    @file ParticleFilter.hpp
    @author Lin, Zhaozhi
    @date Dec 2019
*/

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#define _USE_MATH_DEFINES

#include <cmath>
#include <ctime>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

class MapReader {
public:
    /** Constructor
    Read the map data represented in a text file
    The first 7 lines are headers with identifiers;
    The remaining lines represents an occupancy grid map.
    Note that the value of each grid stands for:
        - 1 - Free
        - 0 - Occupied
        - Negative value - Unknow
    It might be different from common occupancy grid map
    where 0 stands for "Free" and 1 stands for "Occupied".
    @param filename the path to the map data file
    */
    MapReader(const std::string& filename);
    /** Getter of the map data
    @return map data
    */
    cv::Mat getMap(void) const;
    /** Getter of the map resolution
    @return resolution of the map
    */
    size_t getResolution(void) const;

private:
    cv::Mat occupancyMap;
    size_t resolution;
};

class MotionModel {
public:
    /** Constructor
    Reference: Sebastian Thrun, Wolfram Burgard & Dieter Fox. Probabilistic Robotics
               Mit Press, 2005
               [Chapter 5.4]
    @param alpha1 square root of alpha1 in Table 5.5
    @param alpha2 square root of alpha2 in Table 5.5
    @param alpha3 square root of alpha3 in Table 5.5
    @param alpha4 square root of alpha4 in Table 5.5
    */
    MotionModel(const double& alpha1, const double& alpha2, const double& alpha3, const double& alpha4);
    
    /** Transition the states of the particles.
    Reference: Sebastian Thrun, Wolfram Burgard & Dieter Fox. Probabilistic Robotics
               Mit Press, 2005
               [Table 5.5]
    @param particles list of particles ([x,y,heading,weight]) pending update
    @param pose0 previous reading from odometer, given in [x,y,theta]
    @param pose1 new reading from odometer, given in [x,y,theta]
    */
    void update(std::vector<cv::Vec4d>& particles, const cv::Vec3d& pose0, const cv::Vec3d& pose1);
    
private:
    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    cv::RNG rng;

    cv::Vec3d computeTransformation(const cv::Vec3d& pose0, const cv::Vec3d& pose1);
    cv::Vec3d computeStandardDeviation(const cv::Vec3d& transformations);
};

class SensorModel {
public:
    /** Constructor
    Reference: Sebastian Thrun, Wolfram Burgard & Dieter Fox. Probabilistic Robotics
               Mit Press, 2005
               [Chapter 6.3]
    @param zHit "weight" of correct range, Chapter 6.3.1
    @param zShort "weight" of unexpected objects, refer Chapter 6.3.1
    @param zMax "weight" of missed, refer Chapter 6.3.1
    @param zRand "weight" of random measurements, refer Chapter 6.3.1
    @param varHit variance of measurement noise, refer Equation (6.4) and (6.5)
    @param rateShort inverse scale of unexpected-object measurement, refer Equation (6.7)
    @param range the range of the sensor
    */
    SensorModel(const double zHit, const double zShort, const double zMax, const double zRand, const double varHit, const double rateShort, const double range);
    
    /** Update the weights the particles.
    Reference: Sebastian Thrun, Wolfram Burgard & Dieter Fox. Probabilistic Robotics
               Mit Press, 2005
               [Table 6.1]
    @param particles list of particles ([x,y,heading,weight]) pending update
    @param sensorOffset offset of the sensor from the robot center
    @param measurements mapping from the angles (in rad) to measurements
    @param occupancyMap the occupancy grid map in below format
        - 1 - Free
        - 0 - Occupied
        - Negative value - Unknow
    @param threshold the above which a grid is considered free
    @param displayMap the colored map used for visualization
    */
    void update(std::vector<cv::Vec4d>& particles, const double sensorOffset, std::unordered_map<double, double> measurements, const cv::Mat& occupancyMap, const float threshold, cv::Mat& displayMap);
private:
    double zHit;
    double zShort;
    double zMax;
    double zRand;
    
    double range;
    double rateShort;
    double varHit;

    double rayCasting(const double sensorX, const double sensorY, const double heading,
        const cv::Mat& occupancyMap,
        const float threshold,
        cv::Mat& displayMap);
    double computeLikelihood(const double actual, const double expected);
};

/** Initialize particles in the free space.
    @param occupancyMap the occupancy grid map
    @param 
    @param measurements mapping from the angles (in rad) to measurements
    @param occupancyMap the occupancy grid map in below format
        - 1 - Free
        - 0 - Occupied
        - Negative value - Unknow
    @param numParticles number of particles to draw from the space
    @return sampled particles
    */
std::vector<cv::Vec4d> sample(const cv::Mat& occupancyMap, const size_t numParticles);

/** Resample the particles based on the updated weights.
    Reference: Sebastian Thrun, Wolfram Burgard & Dieter Fox. Probabilistic Robotics
               Mit Press, 2005
               [Table 4.4]
    @param particles the updated particles pending resampling
    @return resampled particles
    */
std::vector<cv::Vec4d> resample(std::vector<cv::Vec4d>& particles);

#endif