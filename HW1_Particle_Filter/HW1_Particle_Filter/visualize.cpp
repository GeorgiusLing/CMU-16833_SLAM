#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "data.hpp"

int main(void) {
	Mapper mapper;
	std::string winname = "Particle Filter Demo";
	mapper.readMap("wean.dat");
	cv::namedWindow(winname, cv::WINDOW_AUTOSIZE);
	mapper.drawMap(winname);
	while (cv::waitKey(0) != 27);
	system("pause");
	return 0;
}