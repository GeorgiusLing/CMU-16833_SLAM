#include <string>
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "data.hpp"
#include "filter.hpp"

int main(void) {
    
    Mapper mapper;
	// Read map and log
	cv::Mat free = mapper.readMap("../../data/wean.dat");
    std::vector<RobotDatum> data = readLog("../../data/robotdata4.log");

    std::string winname = "Particle Filter Demo";
	cv::namedWindow(winname, cv::WINDOW_AUTOSIZE);

    ParticleFilter filter;
    // Configure sensor model
    filter.sensor_model.param_hit = 0.3480 * 1000;
    filter.sensor_model.param_max = 0.002 * 1000;
    filter.sensor_model.param_rand = 0.625 * 1000;
    filter.sensor_model.param_short = 0.025 * 1000;
    filter.sensor_model.lambda_short = 0.003;
    filter.sensor_model.sigma_hit = 200;
    filter.sensor_model.range = 8192;
    // Configure motion model
    filter.odometry_model.param1 = 0.05;
    filter.odometry_model.param2 = 0.04;
    filter.odometry_model.param3 = 0.05;
    filter.odometry_model.param4 = 0.07;

    filter.setMap(free, 0.7);
    filter.sample(100);


    for (auto index = 0; index < data.size(); ++index) {
        
        filter.update(data[index]);
        filter.resample();
        filter.visualize(winname, 0.1, 0.1);
    }
	while (cv::waitKey(0) != 27);

    exit(EXIT_SUCCESS);
}