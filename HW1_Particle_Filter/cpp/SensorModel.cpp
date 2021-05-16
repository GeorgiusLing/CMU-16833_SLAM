/**
    16833 Assignment 1: Particle Filter (re-written)
    @file SensorModel.cpp
    @author Lin, Zhaozhi
    @date Dec 2019
*/


#include "ParticleFilter.hpp"

const cv::Scalar rayColor(0.0, 1.0, 0.0);

SensorModel::SensorModel(const double zHit, const double zShort, const double zMax, const double zRand, const double varHit, const double rateShort, const double range) {
    this->range = range;
    this->zHit = zHit;
    this->zShort = zShort;
    this->zMax = zMax;
    this->zRand = zRand;
    this->rateShort = rateShort;
    this->varHit = varHit;
}

double SensorModel::computeLikelihood(const double actual, const double expected) {
    auto normalizerShort = (1.0 - std::exp(-this->rateShort*expected));
    auto normalizerHit = (std::erfc(-((actual - expected) / std::sqrt(2.0 * this->varHit))) 
        - std::erfc(expected / std::sqrt(2.0 * this->varHit))) / 2.0;

    auto pHit = std::exp(-(actual - expected) * (actual - expected) / (2.0 * this->varHit)) / std::sqrt(2.0*M_PI*this->varHit) / normalizerHit;
    auto pRand = (actual < this->range ? 1.0 / this->range : 0.0);
    auto pMax = (actual == this->range);
    auto pShort = (actual < expected ? std::exp(-this->rateShort*actual) / normalizerShort : 0.0);
    return this->zHit * pHit + this->zRand * pRand + this->zMax * pMax + this->zShort * pShort;
}

double SensorModel::rayCasting(const double sensorX, const double sensorY, const double heading, 
    const cv::Mat& occupancyMap,
    const float threshold,
    cv::Mat& displayMap) {

    cv::Point2d coord0(sensorX, sensorY);
    cv::Point2d coord1(sensorX + this->range * std::cos(heading), sensorY + this->range * std::sin(heading));
    cv::LineIterator itr(occupancyMap, coord0, coord1);

    for (int index = 0; index < itr.count; ++index, ++itr) {
        float inoccupancy = *(const float*)*itr;
        if (inoccupancy < threshold) {
            cv::Point obstacle = itr.pos();
            cv::line(displayMap, coord0, obstacle, rayColor);
            return std::hypot(obstacle.x - coord0.x, obstacle.y - coord0.y);
        }
    }
    cv::line(displayMap, coord0, coord1, rayColor);
    return this->range;
}

void SensorModel::update(std::vector<cv::Vec4d>& particles, 
    const double sensorOffset, 
    const std::unordered_map<double, double> measurements, 
    const cv::Mat& occupancyMap, 
    const float threshold, 
    cv::Mat& displayMap) {

    cv::parallel_for_(cv::Range(0, particles.size()), [&](const cv::Range& range) {
        for (auto index = range.start; index < range.end; ++index) {
            auto sensorX = particles[index][0] + sensorOffset * std::cos(particles[index][2]);
            auto sensorY = particles[index][1] + sensorOffset * std::sin(particles[index][2]);
            particles[index][3] = 0.0;
            for (auto& measurement : measurements) {
                auto angle = measurement.first;
                auto expected = this->rayCasting(sensorX, sensorY, particles[index][2] + angle, occupancyMap, threshold, displayMap);
                auto actual = measurement.second;
                particles[index][3] += std::log2(this->computeLikelihood(actual, expected));
            }
            particles[index][3] = std::exp2(particles[index][3]);
            //std::cout << "Weight " << particles[index][3] << std::endl;
        }
    });

    /* Sequential version of above block
    for (auto& particle : particles) {
        
        auto sensorX = particle[0] + sensorOffset * std::cos(particle[2]);
        auto sensorY = particle[1] + sensorOffset * std::sin(particle[2]);
        particle[3] = 0.0;
        for (auto& measurement : measurements) {
            auto angle = measurement.first;
            auto expected = this->rayCasting(sensorX, sensorY, particle[2] + angle, occupancyMap, threshold, displayMap);
            auto actual = measurement.second;
            particle[3] += std::log2(this->computeLikelihood(actual, expected));
        }
        particle[3] = std::exp2(particle[3]);
    }
    */
}