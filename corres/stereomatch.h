#include "iostream"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class StereoMatcher {
private:
	StereoBM BMState;
	Mat map1x, map2x, map1y, map2y;
	Mat gray1, gray2;
	Mat disp;

public:
	StereoMatcher();
	Mat& getDisparity(Mat& img1, Mat& img2);
};