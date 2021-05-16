/**
    16833 Assignment 1: Particle Filter (re-written)
    @file sampling.cpp
    @author Lin, Zhaozhi
    @date Dec 2019
*/


#include "ParticleFilter.hpp"

cv::RNG rng(time(nullptr));

std::vector<cv::Vec4d> sample(const cv::Mat& occupancyMap, const size_t numParticles) {
    std::vector<cv::Vec4d> particles;
    
    while (particles.size() < numParticles) {
        double x = rng.uniform(0.0, occupancyMap.cols - 0.0);
        double y = rng.uniform(0.0, occupancyMap.rows - 0.0);
        if (occupancyMap.at<float>((int)y, (int)x) != 1.0) {
            continue;
        }
        double heading = rng.uniform(0.0, 2 * M_PI - 0.01);
        cv::Vec4d particle(x, y, heading, 1.0 / numParticles);
        particles.push_back(particle);
    }
    return particles;
}

std::vector<cv::Vec4d> resample(std::vector<cv::Vec4d>& particles) {

    double weightSum = std::accumulate(particles.begin(), particles.end(), 0.0,
        [&](cv::Vec4d particle1, cv::Vec4d particle2) {return particle1[3] + particle2[3]; });
    std::for_each(particles.begin(), particles.end(), [&weightSum](cv::Vec4d &particle) {particle[3] = particle[3] / weightSum; });
    std::vector<cv::Vec4d> resampled;

    double accumuWeight = particles[0][3];
    double tooth = rng.uniform(0.0, 1.0 / particles.size()); // The "comb tooth"
    size_t index = 0;

    while (resampled.size() < particles.size()) {
        while (accumuWeight < tooth) {
            ++index;
            accumuWeight += particles[index][3];
        }
        resampled.push_back(cv::Vec4d(particles[index]));
    }
    return resampled;
}