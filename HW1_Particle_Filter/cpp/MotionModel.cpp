/**
    16833 Assignment 1: Particle Filter (re-written)
    @file MotionModel.cpp
    @author Lin, Zhaozhi
    @date Dec 2019
*/


#include "ParticleFilter.hpp"

MotionModel::MotionModel(const double& alpha1, 
    const double& alpha2, 
    const double& alpha3, 
    const double& alpha4) {
    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
    this->alpha3 = alpha3;
    this->alpha4 = alpha4;
    this->rng = cv::RNG(time(nullptr));
}

cv::Vec3d MotionModel::computeTransformation(const cv::Vec3d& pose0, const cv::Vec3d& pose1) {
    const cv::Vec3d diff = pose1 - pose0;
    double rot1 = std::atan2(diff[1], diff[0]) - pose0[2];
    double trans = std::hypot(diff[0], diff[1]);
    double rot2 = diff[2] - rot1;
    return cv::Vec3d(rot1, trans, rot2);
}

cv::Vec3d MotionModel::computeStandardDeviation(const cv::Vec3d& transformations) {

    double rot1 = transformations[0];
    double trans = transformations[1];
    double rot2 = transformations[2];

    double stddevRot1 = std::hypot(this->alpha1 * rot1, this->alpha2 * trans);
    double stddevTrans = this->alpha4 * std::hypot(rot1, rot2);
    stddevTrans = std::hypot(stddevTrans, this->alpha3 * trans);
    double stddevRot2 = std::hypot(this->alpha1 * rot2, this->alpha2 * trans);
    return cv::Vec3d(stddevRot1, stddevTrans, stddevRot2);
}

void MotionModel::update(std::vector<cv::Vec4d>& particles, const cv::Vec3d& pose0, const cv::Vec3d& pose1) {
    auto transformations = this->computeTransformation(pose0, pose1);
    auto stddev = this->computeStandardDeviation(transformations);

    double rot1 = transformations[0];
    double trans = transformations[1];
    double rot2 = transformations[2];

    double stddevRot1 = stddev[0];
    double stddevTrans = stddev[1];
    double stddevRot2 = stddev[2];

    cv::parallel_for_(cv::Range(0, particles.size()), [&](const cv::Range& range) {
        for (auto index = range.start; index < range.end; ++index) {
            double rot1Noisy = rot1 + rng.gaussian(stddevRot1);
            double transNoisy = trans + rng.gaussian(stddevTrans);
            double rot2Noisy = rot2 + rng.gaussian(stddevRot2);

            particles[index][2] += rot1Noisy;
            particles[index][0] += transNoisy * std::cos(particles[index][2]);
            particles[index][1] += transNoisy * std::sin(particles[index][2]);
            particles[index][2] += rot2Noisy;
        }
    });

    /* Sequential version of above block
    for (auto& particle : particles) {

        double rot1Noisy = rot1 + rng.gaussian(stddevRot1);
        double transNoisy = trans + rng.gaussian(stddevTrans);
        double rot2Noisy = rot2 + rng.gaussian(stddevRot2);

        particle[2] += rot1Noisy;
        particle[0] += transNoisy * std::cos(particle[2]);
        particle[1] += transNoisy * std::sin(particle[2]);
        particle[2] += rot2Noisy;
    }
    */
}