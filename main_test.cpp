#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"

#define FRAME_H 240
#define FRAME_W 320

using namespace cv;

int main(int argc, char** argv) {
	VideoCapture cap1(1);
	VideoCapture cap2(2);

    if (!cap1.isOpened() || !cap2.isOpened()) {
        std::cout << "ERROR: Cameras aren't connected properly..\n";
        return 1;
    }

    cap1.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_W);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_H);
    cap2.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_W);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_H);


	Mat img1, img2, img_1, img_2, map1x, map1y, map2x, map2y;
	FileStorage fs("stereorect.xml", FileStorage::READ);
	fs["map1x"] >> map1x;
	fs["map2x"] >> map2x;
	fs["map1y"] >> map1y;
	fs["map2y"] >> map2y;
	fs.release();

    while (1) {
        cap1 >> img1; cap2 >> img2;
        remap(img1, img_1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    	remap(img2, img_2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    	int rows = img1.rows, cols = img1.cols;
    	for (int i = 40; i<rows; i+=40) {
        	line(img1, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
        	line(img2, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
        	line(img_1, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
        	line(img_2, Point(0, i), Point(cols, i), Scalar(0, 255, 0), 2);
    	}
        imshow("original 1", img1);
        imshow("original 2", img2);
        imshow("rectified 1", img_1);
        imshow("rectified 2", img_2);
        int k = waitKey(0);
        if (k==27) break;
    }
    
	return 0;
}