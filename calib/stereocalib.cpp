#include "stereocalib.h"

StereoCalibrator::StereoCalibrator(int num, int bw, int bh, float obj_scale) {
	num_boards = num;
	b_width = bw;
	b_height = bh;
	ob_scale = obj_scale;
	b_size = cv::Size(b_width, b_height);
	M1 = cv::Mat(3, 3, CV_64FC1);
	M2 = cv::Mat(3, 3, CV_64FC1);

	int bn = b_width*b_height;
	for (int i=0; i<bn; i++)
		obj.push_back(cv::Point3f(ob_scale*(i/b_width), ob_scale*(i%b_width), 0.0f));

	std::cout << "VERBOSE: Object Matrix for chessboard :\n";
	std::cout << obj << "\n";
}

int StereoCalibrator::detectChessboardCorners(cv::Mat& img1, cv::Mat& img2) {
	this -> img1 = img1;
	this -> img2 = img2;
	cv::cvtColor(img1, gray1, CV_BGR2GRAY);
	cv::cvtColor(img2, gray2, CV_BGR2GRAY);

	bool res1 = cv::findChessboardCorners(gray1, b_size, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK ); 
	bool res2 = cv::findChessboardCorners(gray2, b_size, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK );

	if (res1 && res2) {
		cv::cornerSubPix(gray1, corners1, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(gray1, b_size, corners1, res1);
        cv::cornerSubPix(gray2, corners2, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(gray2, b_size, corners2, res2);
	}

	cv::imshow("FRAME 1", gray1);
	cv::imshow("FRAME 2", gray2);
	if (cv::waitKey(33)==27) return -1;

	int ret = 0;
	if (res1 && res2) {
		if (cv::waitKey(0)==' ') {
			imgPts1.push_back(corners1);
            imgPts2.push_back(corners2);
            objPts.push_back(obj);
            ret = 1;
        }
	}

	return ret;
}

void StereoCalibrator::calibrate() {
	std::cout << "STATUS: Calibration started ..\n";
	cv::stereoCalibrate(objPts, imgPts1, imgPts2, 
                    M1, D1, M2, D2, img1.size(), R, T, E, F,
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
	std::cout << "STATUS: Calibration done ..\n";
	//testing
	//std::cout << "Image Points 1:\n";
	//for (int k=0; k<imgPts1.size(); k++) std::cout<< imgPts1[k] << "\n";
	//std::cout << "Image Points 2:\n" << imgPts2 << "\n";

	char update = 'n';
	std::cout << "Update stereocalib.xml? (y/n): ";
	std::cin >> update;
	if (update == 'y' || update == 'Y') {
		cv::FileStorage fs1("stereocalib.xml", cv::FileStorage::WRITE);
		std::cout << "STATUS: Updating stereocalib.xml..\n";
		fs1 << "M1" << M1;
    	fs1 << "M2" << M2;
    	fs1 << "D1" << D1;
    	fs1 << "D2" << D2;
    	fs1 << "R" << R;
    	fs1 << "T" << T;
    	fs1 << "E" << E;
    	fs1 << "F" << F;
    	fs1.release();
	}
	std::cout << "VERBOSE: showing calibrated params..\n";
	std::cout << "Camera Matrix 1 :\n" << M1 << std::endl;
	std::cout << "Camera Matrix 2 :\n" << M2 << std::endl;
	std::cout << "Distortion Matrix 1:\n" << D1 << std::endl;
	std::cout << "Distortion Matrix 2:\n" << D2 << std::endl;
	std::cout << "Rotation Matrix:\n" << R << std::endl;
	std::cout << "Translation Vector:\n" << T << std::endl;
	std::cout << "Essential Matrix:\n" << E << std::endl;
	std::cout << "Fundamental Matrix:\n " << F << std::endl;
}
