#include "./corres/stereomatch.h"

#define FRAME_H 240
#define FRAME_W 320

int startx, starty, endx, endy;
bool drawing = false;
Mat disp, img1, img2;
StereoMatcher* sm;

static void onMouse(int event, int x, int y, int flags, void*);

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

	namedWindow("disparity", CV_WINDOW_AUTOSIZE);
	setMouseCallback("disparity", onMouse);
	sm = new StereoMatcher();

	while (1) {
		cap1 >> img1;
		cap2 >> img2;
		disp = sm -> getDisparity(img1, img2);
		dilate(disp, disp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		
		sm -> reproject(disp, img1);
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

static void onMouse(int event, int x, int y, int flags, void*) {
	if (event==EVENT_LBUTTONDOWN) {
		drawing = true;
		startx = x; starty = y;
	}
	else if (event == EVENT_LBUTTONUP) {
		drawing = false;
		endx = x; endy = y;
		Mat cropped = disp(Rect(startx, starty, abs(startx-endx), abs(starty-endy)));
		cout << "mean: " << mean(cropped)[0] <<"\n";
		rectangle(disp, Point(startx, starty), Point(x, y), Scalar(0, 0, 255), 1);
		imshow("disparity", disp);
		sm->getDepth(startx, starty, endx, endy);
	}
}