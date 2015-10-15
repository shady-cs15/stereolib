#include "stereomatch.h"

StereoBlockMatcher::StereoBlockMatcher() {
	bm = StereoBM::create(128, 41);
	bm->setPreFilterSize(41);
	bm->setPreFilterCap(31);
	bm->setBlockSize(41);
	bm->setMinDisparity(64);
	bm->setNumDisparities(128);
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(15);

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

Mat& StereoBlockMatcher::getDisparity(Mat& img1, Mat& img2) {
	remap(img1, img1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
	remap(img2, img2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

	cvtColor(img1, gray1, CV_BGR2GRAY);
	cvtColor(img2, gray2, CV_BGR2GRAY);

	bm->compute(gray1, gray2, disp);
	normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
	return disp;
}

void StereoBlockMatcher::reproject(Mat& disp, Mat& img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ptr) {
	double px, py, pz, pw;
	unsigned char pb, pg, pr;
	for (int i=0;i<img.rows;i++) {
        uchar* rgb = img.ptr<uchar>(i);
        for (int j=0;j<img.cols;j++) {
            int d = static_cast<unsigned>(disp(Rect(j, i, 1, 1)).at<uchar>(0));
            if (d==0) continue;
            double pw = -1.0*(double) (d)*_tx_inv + _cx_cx_tx_inv;
            px = static_cast<double> (j) + _cx;
            py = static_cast<double> (i) + _cy;
            pz = f;

            px/=pw; 
            py/=pw;
            pz/=pw; pz*=-1; //pz inverted

            pb = rgb[3*j];
            pg = rgb[3*j+1];
            pr = rgb[3*j+2];

            pcl::PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;

            uint32_t _rgb = ((uint32_t) pr << 16 |
              (uint32_t) pg << 8 | (uint32_t)pb);
            point.rgb = *reinterpret_cast<float*>(&_rgb);
            ptr->push_back(point);
        }
    }
    ptr->width = (int) ptr->points.size();
    ptr->height = 1;
}

double StereoBlockMatcher::getDepth(int startx, int starty, int endx, int endy) {
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