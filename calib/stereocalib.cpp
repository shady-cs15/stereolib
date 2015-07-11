#include "stereocalib.h"

StereoCalibrator::StereoCalibrator(int num, int bw, int bh, float obj_scale) {
	num_boards = num;
	b_width = bw;
	b_height = bh;
	ob_scale = obj_scale;
	b_size = cv::Size(b_width, b_height);

	int bn = b_width*b_height;
	for (int i=0; i<bn; i++)
		obj.push_back(cv::Point3f(ob_scale*(i/b_width), ob_scale*(i%b_width), 0.0f));

	//verbose
	std::cout << "Object Matrix for chessboard :\n";
	std::cout << obj << "\n";
}

int StereoCalibrator::detectChessboardCorners(cv::Mat& img1, cv::Mat& img2) {
	cv::cvtColor(img1, gray1, CV_BGR2GRAY);
	cv::cvtColor(img2, gray2, CV_BGR2GRAY);

	bool res1 = cv::findChessboardCorners(gray1, b_size, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK | CV_CALIB_CB_FILTER_QUADS);
	bool res2 = cv::findChessboardCorners(gray2, b_size, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK | CV_CALIB_CB_FILTER_QUADS);

	if (res1 && res2) {
		cv::cornerSubPix(gray1, corners1, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(gray1, b_size, corners1, res1);
        cv::cornerSubPix(gray2, corners2, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(gray2, b_size, corners2, res2);
	}

	cv::imshow("FRAME 1", gray1);
	cv::imshow("FRAME 2", gray2);
	cv::waitKey(33);

	int ret = 0;
	if (res1 && res2) {
		if (cv::waitKey(0)==' ') {
			imgPts1.push_back(corners1);
            imgPts2.push_back(corners2);
            objPts.push_back(obj);
            ret = 1;
            std::cout << "Image points pushed..\n";
		}
	}

	return ret;
}

void StereoCalibrator::calibrate(bool update) {
	
}