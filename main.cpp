#include "./calib/stereocalib.h"

#define FRAME_H 480
#define FRAME_W 640

int main(int argc, char** argv) {
	int nboards = atoi(argv[1]);
	int rows = atoi(argv[2]);
	int cols = atoi(argv[3]);
	float bscale = atof(argv[4]); 

	StereoCalibrator* sc = new StereoCalibrator(nboards, rows, cols, bscale);

	cv::VideoCapture cap1(1);
	cv::VideoCapture cap2(2);
	cv::Mat img1;
	cv::Mat img2;

	if (!cap1.isOpened() || !cap2.isOpened()) {
		std::cout << "ERROR: Cameras aren't connected properly..\n";
		return 1;
	}

	cap1.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_W);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_H);
    cap2.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_W);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_H);

    int chessboard_frames = 0;
    std::cout << "STATUS: Started Chessboard pattern detection\n";
    while (chessboard_frames < nboards) {
    	cap1 >> img1; cap2 >> img2;
    	int r = sc->detectChessboardCorners(img1, img2);
    	if (r<0) break;
    	if (r==1) {
    		chessboard_frames++;
    	    std::cout << "^[[2K\nBoards pushed: "<<chessboard_frames<<"\n";
		}
    }

    cv::destroyAllWindows();

    if (chessboard_frames >= nboards) {
    	sc -> calibrate();
    }
	return 0;
}