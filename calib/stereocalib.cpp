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
                    cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), 
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);
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

//bug fix it. follow OpenCV text.
/*void StereoCalibrator::check_quality() {
	std::vector<cv::Point3f> l1, l2;
	int n = num_boards*b_width*b_height;
	std::cout << "imgPts1[0]\n" << imgPts1[0] << "\n";
	std::vector<std::vector<cv::Point2f> > _imgPts1 = imgPts1, _imgPts2 = imgPts2;
	cv::Mat i1(1, n, CV_32FC2, &_imgPts1[0]);
	cv::Mat i2(1, n, CV_32FC2, &_imgPts2[0]);
	//TODO- needs to be removed from here
	cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(M1, D1, M2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
	/*std::vector<cv::Point2f> i1, i2;
	int n = imgPts1.size();
	for (int i = 0; i<n; i++) {
		i1.push_back(cv::Point2f(imgPts1[i][0], imgPts1[i][1]));
		i2.push_back(cv::Point2f(imgPts2[i][0], imgPts2[i][1]));
	}*/
	/*std::cout << "i1:\n" << i1 << "\n";
	std::cout << "i1.size(): " << i1.size() << "\n";
	cv::undistortPoints( i1, i1, M1, D1, R1, M1);
	std::cout << "i1:\n" << i1 << "\n";
	cv::undistortPoints( i2, i2, M2, D2, R2, M2);
	cv::computeCorrespondEpilines( _imgPts1, 1, F, l1);
	cv::computeCorrespondEpilines( _imgPts2, 2, F, l2);
	double avg_err = 0;
	std::cout << "imgPts1[0]\n" << imgPts1[0] << "\n"; //probably giving error
	std::cout << "l1:\n" << l1 << "\n";
	std::cout << "l1.size(): " << l1.size() << "\n";
	for (int i = 0; i<2; i++) {
		for (int j=0; j<54; j++) {
			int k = i*54+j;
			//if (l1[k]!=l1[k] || l2[k]!=l2[k]) continue; //if either is NaN continue;
			/*double err = fabs(imgPts1[i][j].x*l2[k].x+imgPts1[i][j].x*l2[k].y+l2[k].z)
						+fabs(imgPts2[i][j].x*l1[k].x+imgPts2[i][j].y*l1[k].y+l1[k].z);
			avg_err += err;*/
	/*	}
	}
	std::cout << "Average calibration error: " << avg_err/(num_boards*b_width*b_height);
}*/