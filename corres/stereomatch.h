#include "iostream"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace cv;
using namespace cuda;

class StereoBlockMatcher {
private:
	Ptr<StereoBM> bm;
	double min_, max_;
	Mat map1x, map2x, map1y, map2y, Q;
	Mat gray1, gray2;
	Mat disp16S, disp8U;
	double _cx, _cy, f, _tx_inv, _cx_cx_tx_inv;

public:
	StereoBlockMatcher();
	Mat& getDisparity(Mat& img1, Mat& img2);
	void reproject(Mat& disp, Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr);
	double getDepth(int startx, int starty, int endx, int endy);
};