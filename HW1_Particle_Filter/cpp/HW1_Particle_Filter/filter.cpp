#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <string>
#include <functional>
#include <numeric>
#include <limits>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "filter.hpp"

static double normalPDF(double x, double mean = 0.0, double stddev = 1.0) {
    double y = std::exp(-0.5 * (x - mean) * (x - mean) / stddev / stddev);
    y /= stddev;
    y /= std::sqrt(2 * M_PI);
    return y;
}

static double exponentialPDF(double x, double x_max, double rate) {
    double y = 0.0;
    if (x >= 0 && x < x_max) {
        y = rate * std::exp(-rate * x);
    }
    return y;
}

static double pointPDF(double x, double x_mass) {
    return (x == x_mass);
}

static double uniformPDF(double x, double x_min, double x_max) {
    double y = 0.0;
    if (x >= x_min && x <= x_max) {
        y = 1.0 / (x_max - x_min);
    }
    return y;
}

MotionModel::MotionModel(void) {
    this->displacement = 0;
    this->rotation1 = 0;
    this->rotation2 = 0;
    this->timestamp = -1;

    this->param1 = 0.05;
    this->param2 = 0.04;
    this->param3 = 0.05;
    this->param4 = 0.07;
}

void MotionModel::updatePose(cv::Vec4d &particle) {
    double rotation1_noisy = rotation1 - rng.gaussian(std::hypot(param1 * rotation1, param2 * displacement));
    double displacement_noisy = displacement - rng.gaussian(std::hypot(param3 * displacement, param4 * std::hypot(rotation1, rotation2)));
    double rotation2_noisy = rotation2 - rng.gaussian(std::hypot(param1 * rotation2, param2 * displacement));

    particle[2] += rotation1_noisy;
    particle[0] += displacement_noisy * std::cos(particle[2]);
    particle[1] += displacement_noisy * std::sin(particle[2]);
    particle[2] += rotation2_noisy;

}

void MotionModel::updateOdom(RobotDatum &datum) {
    if (datum.timestamp <= this->timestamp) {
        return; 
    }
    if (this->timestamp < 0) {
        this->pose = datum.pose0;
        this->timestamp = datum.timestamp;
        return;
    }
    cv::Vec3d diff = datum.pose0 - this->pose;
    displacement = std::hypot(diff[0], diff[1]);
    rotation1 = std::atan2(diff[1], diff[0]) - this->pose[2];
    rotation2 = diff[2] - rotation1;

    this->pose = datum.pose0;
    this->timestamp = datum.timestamp;
}

void SensorModel::updateWeight(const cv::Mat &inoccupancy, const double &threshold, cv::Vec4d &particle, std::vector<int> measurements, int step) {
    particle[3] = 0;
    double angle = particle[2] - M_PI_2;
    for (int index = 0; index < 180; index += step) {
        cv::Point2d unit(std::cos(angle), std::sin(angle));
        cv::Point2d end(particle[0], particle[1]);
        int length = -1;
        while (++length < range) {
            if (end.x < 0 || end.y < 0 || end.x >= inoccupancy.cols || end.y >= inoccupancy.rows || inoccupancy.at<float>((int)end.y, (int)end.x) < 0) {
                length = range;
                break;
            }
            else if (inoccupancy.at<float>((int)end.y, (int)end.x) <= threshold) {
                break;
            }
            end += unit;
        }
        particle[3] += calcLikelohood(measurements[index], length);
        angle += (step * M_PI / 180);
    }
    particle[3] = std::pow(2, particle[3]);
    std::cout << "Weight: " << particle[3] << std::endl;
}

double SensorModel::calcLikelohood(int reading, double expected) {
    double likelihood = param_hit * normalPDF(reading, expected, sigma_hit)
        + param_short * exponentialPDF(reading, expected, lambda_short)
        + param_max * pointPDF(reading, range)
        + param_rand * uniformPDF(reading, 0.0, range);
    likelihood = std::max(std::log2(likelihood), -DBL_MAX / 100000);
    return likelihood;
}


void ParticleFilter::setMap(cv::Mat &inoccupancy, float threshold) {
    this->inoccupancy = inoccupancy;
    this->threshold = threshold;
}

void ParticleFilter::sample(unsigned int num_particles) {
    cv::RNG rng;
    this->comb.clear();
    this->comb.push_back(rng.uniform(0.0, 1.0 / num_particles));

    while (this->particles.size() < num_particles) {
        double x = rng.uniform(0, inoccupancy.cols - 1);
        double y = rng.uniform(0, inoccupancy.rows - 1);
        if (inoccupancy.at<float>((int)y, (int)x) < threshold) {
            continue;
        }
        double heading = rng.uniform(0.0, 2 * M_PI - 0.01);
        cv::Vec4d particle(x, y, heading, 1.0 / num_particles);
        this->particles.push_back(particle);
        this->comb.push_back(comb.back() + 1.0 / num_particles);
    }
    comb.pop_back();
}

void ParticleFilter::update(RobotDatum &datum) {

    this->odometry_model.updateOdom(datum);
#pragma omp parallel for
    for (int i = 0; i < particles.size(); ++i) {
        this->odometry_model.updatePose(particles[i]);
        if (datum.type == 'L') {
            this->sensor_model.updateWeight(inoccupancy, this->threshold, particles[i], datum.measurements, 3);
        }
    }
}

void ParticleFilter::resample(void) {
    double weight_sum = std::accumulate(particles.begin(), particles.end(), 0.0, 
        [](double total, cv::Vec4d &particle) { return total + particle[3]; });
    std::for_each(particles.begin(), particles.end(), [&weight_sum](cv::Vec4d &particle) {particle[3] /= weight_sum; });

    double cumulative_weight = 0.0;
    int particle_index = -1;
    int comb_index = 0;

    while (comb_index < particles.size()) {
        
        if (comb[comb_index] > cumulative_weight) {
            particle_index++;
            cumulative_weight += particles[particle_index][3];
        }
        else {
            particles[comb_index][0] = particles[particle_index][0];
            particles[comb_index][1] = particles[particle_index][1];
            particles[comb_index][2] = particles[particle_index][2];
            ++comb_index;
        }
    }
    
}

void ParticleFilter::visualize(std::string winname, double fx, double fy) {
    cv::Mat display;
    cv::cvtColor(inoccupancy, display, cv::COLOR_GRAY2RGB);
#pragma omp parallel for
    for (int i = 0; i < particles.size(); ++i) {
        cv::ellipse(display, cv::Point2d(particles[i][0], particles[i][1]), axes, particles[i][2] * 180 * M_1_PI, 0, 360, color, -1);
    }
    cv::resize(display, display, cv::Size(0, 0), fx, fy);
    cv::imshow(winname, display);
    cv::waitKey(1);
}