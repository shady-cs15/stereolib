#include "stereomatch.h"

StereoMatcher::StereoMatcher() {
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
	fs["Q"]>>Q;
	fs.release();

	cout << "Q:\n" << Q << "\n";
	_cx = Q.at<double>(0, 3);
	_cy = Q.at<double>(1, 3);
	f = Q.at<double>(2, 3);
	_tx_inv = Q.at<double>(3, 2);
	_cx_cx_tx_inv = Q.at<double>(3, 3);
}

Mat& StereoMatcher::getDisparity(Mat& img1, Mat& img2) {
	remap(img1, img1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	remap(img2, img2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	cvtColor(img1, gray1, CV_BGR2GRAY);
	cvtColor(img2, gray2, CV_BGR2GRAY);

	BMState(gray1, gray2, disp);
	normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
	return disp;
}

void StereoMatcher::reproject(Mat& disp, Mat& img) {
	double px, py, pz, pw;
	unsigned char pb, pg, pr;
	for (int i=0;i<img.rows;i++) {
        uchar* rgb = img.ptr<uchar>(i);
        for (int j=0;j<img.cols;j++) {
            int d = static_cast<unsigned>(disp(Rect(j, i, 1, 1)).at<uchar>(0));
            if (d==0) continue;
            double pw = -1.0*(double) (d)*_tx_inv + _cx_cx_tx_inv;
            px = (double) j + _cx;
            py = (double) j + _cy;
            pz = f;

            px/=pw;
            py/=pw;
            pz/=pw;

            pb = rgb[3*j];
            pg = rgb[3*j+1];
            pr = rgb[3*j+2];

            //verbose
            //cout << "i, j: " << i << " " << j << " x, y, z: " << px <<", "<<py<<", "<<pz<<"\n";
            PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;

            uint32_t _rgb = ((uint32_t) pr << 16 |
              (uint32_t) pg << 8 | (uint32_t)pb);
            point.rgb = *reinterpret_cast<float*>(&_rgb);
            //point_cloud_ptr->points.push_back (point);
        
        }
    }
}

double StereoMatcher::getDepth(int startx, int starty, int endx, int endy) {
	double sum = 0; int count = 0;
	double sum_d = 0;
	for (int i = startx; i<=endx; i++) {
		for (int j = starty; j<=endy; j++) {
			int d = static_cast<unsigned>(disp(Rect(i, j, 1, 1)).at<uchar>(0));
            if (d==0) continue;
            sum_d+=d;
            double pw = -1.0*(double)(d)*_tx_inv + _cx_cx_tx_inv; 
            sum+=f/pw;
            count++;
		}
	}
	cout << std::fixed;
	cout << "Average distance: " << sum/count<<"\n";
	cout << "Average disparity: " << sum_d/count << "\n\n";
	return sum/count;
}