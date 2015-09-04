#include "iostream"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/common/common_headers.h>

using namespace std;
using namespace cv;

class StereoMatcher {
private:
	StereoBM BMState;
	Mat map1x, map2x, map1y, map2y, Q;
	Mat gray1, gray2;
	Mat disp;
	double _cx, _cy, f, _tx_inv, _cx_cx_tx_inv;

public:
	StereoMatcher();
	Mat& getDisparity(Mat& img1, Mat& img2);
	void reproject(Mat& disp, Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr);
	double getDepth(int startx, int starty, int endx, int endy);
};