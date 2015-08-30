#include "stereomatch.h"

StereoMatcher::StereoMatcher() {
	//BMState = cvCreateStereoBMState();
	BMState.state->preFilterSize=41;
	BMState.state->preFilterCap=31;
	BMState.state->SADWindowSize=41;
	BMState.state->minDisparity=-64;
	BMState.state->numberOfDisparities=128;
	BMState.state->textureThreshold=10;
	BMState.state->uniquenessRatio=15;

	FileStorage fs("stereorect.xml",FileStorage::READ);
	fs["map1x"]>>map1x;
	fs["map2x"]>>map2x;
	fs["map1y"]>>map1y;
	fs["map2y"]>>map2y;
	fs.release();
}

Mat& StereoMatcher::getDisparity(Mat& img1, Mat& img2) {
	remap(img1, img1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	remap(img2, img2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	cvtColor(img1, gray1, CV_BGR2GRAY);
	cvtColor(img2, gray2, CV_BGR2GRAY);

	//cout << img1.rows << " " << img1.cols << "\n";
	//cout << BMState.state->SADWindowSize << "\n";
	BMState(gray1, gray2, disp);
	normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

	return disp;
}