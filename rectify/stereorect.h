#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"

class StereoRectifier {
public:
	StereoRectifier(cv::Size s);
	void rectify();
private:
	cv::Mat R1, R2, P1, P2, Q;
	cv::Mat M1, M2, D1, D2, R, T;
	cv::Size img_sz;
};