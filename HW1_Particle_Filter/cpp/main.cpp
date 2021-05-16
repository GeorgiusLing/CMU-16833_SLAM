/**
    16833 Assignment 1: Particle Filter (re-written)
    @file main.cpp
    @author Lin, Zhaozhi
    @date Dec 2019
*/
#include "ParticleFilter.hpp"

// Command line arguments
const std::string keys =
    "{m map     |../data/wean.dat          | path to the map data file}"
    "{l log     |../data/robotdata1.log    | path to the log data file  }"
    "{c config  |../config/parameters.json  | path to the config parameters file  }"
;

// Some constants used for visualization
const std::string winname = "Particle Filter";
const cv::Scalar particleColor(0.0, 0.0, 1.0);
const cv::Scalar textColor(0.0, 1.0, 1.0);
const cv::Size particleSize(3, 1);

// Visualize particles and time step
void visualize(std::vector<cv::Vec4d>& particles, 
    const double timestamp, 
    cv::Mat& displayMap) {
    
    cv::parallel_for_(cv::Range(0, particles.size()), [&](const cv::Range& range) {
        
        for (auto index = range.start; index < range.end; ++index) {
            cv::ellipse(displayMap,
                cv::Point2d(particles[index][0], particles[index][1]),
                particleSize,
                particles[index][2],
                0,
                360,
                particleColor,
                cv::FILLED,
                cv::FILLED);
        }
    });
    const std::string displayMessage = "Time " +std::to_string(timestamp);
    cv::putText(displayMap, displayMessage, cv::Point(0, displayMap.rows / 2), cv::FONT_ITALIC, 1, textColor);
}

// Main routine
int main(int argc, const char *argv[]) {

    cv::CommandLineParser parser(argc, argv, keys);
    const std::string mapname = parser.get<std::string>("map");
    const std::string logname = parser.get<std::string>("log");
    const std::string configname = parser.get<std::string>("config");

    // Read map data
    MapReader mapReader(mapname);
    cv::Mat occupancyMap = mapReader.getMap();
    size_t resolution = mapReader.getResolution();

    // Read model parameters
    cv::FileStorage fs;
    fs.open(configname, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("File " + configname + " not found");
    }
    cv::FileNode node = fs["Mappings"];
    size_t numParticles = (int)node["num_particles"];
    double alpha1 = (double)node["alpha1"];
    double alpha2 = (double)node["alpha2"];
    double alpha3 = (double)node["alpha3"];
    double alpha4 = (double)node["alpha4"];

    double zHit = (double)node["z_hit"];
    double zShort = (double)node["z_short"];
    double zMax = (double)node["z_max"];
    double zRand = (double)node["z_rand"];
    double rateShort = (double)node["lambda_short"];
    double varHit = std::pow((double)node["sigma_hit"], 2.0);
    double range = ((double)node["range"]) / resolution;
    fs.release();

    // Initialize particles
    double threshold = 0.65;
    auto particles = sample(occupancyMap, numParticles);

    // Initialize motion model and sensor model
    MotionModel motionModel(alpha1, alpha2, alpha3, alpha4);
    SensorModel sensorModel(zHit, zShort, zMax, zRand, varHit, rateShort, range);

    // Read logs
    std::ifstream ifs(logname);
    if (!ifs) {
        throw std::runtime_error(logname + " is not found.");
    }
    cv::Vec3d pose0;                     // Previous robot pose
    cv::Vec3d pose1(-1, -1, -1);         // Current robot pose 
    std::string line;
    float timestamp = 0.0;
    
    while (std::getline(ifs, line)) {
        cv::Mat displayMap;
        cv::cvtColor(occupancyMap, displayMap, CV_GRAY2BGR);
        std::stringstream linestream(line);
        char identifier;    // A letter to indicate the message type
        linestream >> identifier;
        if (timestamp == 0.0) {
            linestream >> pose1[0] >> pose1[1] >> pose1[2];
            pose1[0] = pose1[0] / resolution;
            pose1[1] = pose1[1] / resolution;
            double reading;
            while (linestream >> reading) {
                timestamp = reading;
            }
        }
        else if (identifier == 'O') {  // Odometry message
            
            pose0[0] = pose1[0];
            pose0[1] = pose1[1];
            pose0[2] = pose1[2];
            linestream >> pose1[0] >> pose1[1] >> pose1[2] >> timestamp;
            pose1[0] = pose1[0] / resolution;
            pose1[1] = pose1[1] / resolution;
            motionModel.update(particles, pose0, pose1);
        }
        else if (identifier == 'L') {  // Laser rangefinder message 
            
            cv::Vec3d pose2;        // Current sensor pose
            pose0[0] = pose1[0];
            pose0[1] = pose1[1];
            pose0[2] = pose1[2];
            linestream >> pose1[0] >> pose1[1] >> pose1[2]  >> pose2[0] >> pose2[1] >> pose2[2];
            pose1[0] = pose1[0] / resolution;
            pose1[1] = pose1[1] / resolution;
            pose2[0] = pose2[0] / resolution;
            pose2[1] = pose2[1] / resolution;
            double reading;
            double angle = -M_PI_2;
            std::unordered_map<double, double> measurements;
            while (linestream >> reading) {
                angle += M_PI / 180.0;
                measurements[angle] = reading / resolution;
            }

            // The last reading is timestamp
            timestamp = measurements[angle] * resolution;
            measurements.erase(angle);

            motionModel.update(particles, pose0, pose1);

            double sensorOffset = cv::norm(pose0 - pose2, cv::NORM_L2);
            sensorModel.update(particles, sensorOffset, measurements, occupancyMap, threshold, displayMap);
            particles = resample(particles);
        }
        else {
            std::cerr << "Unrecognized identifier " << identifier << std::endl;
            continue;
        }
        // Visualization
        visualize(particles, timestamp, displayMap);
        cv::imshow(winname, displayMap);
        cv::waitKey(1);
    }
    system("pause");
    return EXIT_SUCCESS;
}