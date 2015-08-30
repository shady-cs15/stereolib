#include "./corres/stereomatch.h"

#define FRAME_H 240
#define FRAME_W 320

int main(int argc, char** argv) {
	cv::VideoCapture cap1(1);
	cv::VideoCapture cap2(2);

	if (!cap1.isOpened() || !cap2.isOpened()) {
		std::cout << "ERROR: Cameras aren't connected properly..\n";
		return 1;
	}

	cap1.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_W);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_H);
    cap2.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_W);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_H);

	Mat img1, img2, disp;

	StereoMatcher* sm = new StereoMatcher();

	while (1) {
		cap1 >> img1;
		cap2 >> img2;
		disp = sm -> getDisparity(img1, img2);
		dilate(disp, disp, getStructuringElement(MORPH_ELLIPSE, Size(3, 5)));
		imshow("left", img1);
		imshow("right", img2);
		imshow("disparity", disp);
		int k = waitKey(0);
		if (k == 27) break;
	}
	destroyAllWindows();
	delete sm;	

	return 0;
}